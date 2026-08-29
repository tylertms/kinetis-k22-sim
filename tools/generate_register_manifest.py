import argparse
import re
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path


READ = 1
WRITE = 2
FNV_OFFSET = 0xCBF29CE484222325
FNV_PRIME = 0x100000001B3


@dataclass(frozen=True)
class Register:
    address: int
    reset_value: int
    reset_mask: int
    implemented_mask: int
    read_mask: int
    write_mask: int
    w1c_mask: int
    peripheral_index: int
    width: int
    access: int
    peripheral: str
    name: str


def child_text(element, name, default=None):
    child = element.find(name)
    return default if child is None else child.text


def integer(text):
    return int(text.replace("#", "0x"), 0)


def access_bits(value):
    return {
        "read-only": READ,
        "write-only": WRITE,
        "read-write": READ | WRITE,
        "writeOnce": WRITE,
        "read-writeOnce": READ | WRITE,
    }[value]


def peripheral_kind(name):
    if re.fullmatch(r"PORT[A-Z]", name):
        return "PORT"
    if re.fullmatch(r"GPIO[A-Z]", name):
        return "GPIO"
    return re.sub(r"\d+$", "", name)


def field_mask(field):
    offset = integer(child_text(field, "bitOffset"))
    width = integer(child_text(field, "bitWidth"))
    return ((1 << width) - 1) << offset


def flatten_svd(path):
    root = ElementTree.parse(path).getroot()
    device_width = integer(child_text(root, "width", "32"))
    device_access = child_text(root, "access", "read-write")
    peripherals = []
    registers = []
    for peripheral_index, peripheral in enumerate(root.findall("./peripherals/peripheral")):
        peripheral_name = child_text(peripheral, "name")
        if peripheral_name in {"SystemControl", "SysTick", "NVIC"}:
            continue
        peripheral_index = len(peripherals)
        peripherals.append(peripheral_name)
        base = integer(child_text(peripheral, "baseAddress"))
        peripheral_width = integer(child_text(peripheral, "size", str(device_width)))
        peripheral_access = child_text(peripheral, "access", device_access)
        for register in peripheral.findall("./registers/register"):
            name = child_text(register, "name")
            offset = integer(child_text(register, "addressOffset"))
            width = integer(child_text(register, "size", str(peripheral_width)))
            width_mask = (1 << width) - 1
            register_access = child_text(register, "access", peripheral_access)
            reset_value = integer(child_text(register, "resetValue", "0")) & width_mask
            reset_mask = integer(child_text(register, "resetMask", hex(width_mask))) & width_mask
            implemented_mask = 0
            read_mask = 0
            write_mask = 0
            fields = register.findall("./fields/field")
            if fields:
                for field in fields:
                    mask = field_mask(field) & width_mask
                    implemented_mask |= mask
                    field_access = access_bits(child_text(field, "access", register_access))
                    if field_access & READ:
                        read_mask |= mask
                    if field_access & WRITE:
                        write_mask |= mask
            else:
                implemented_mask = reset_mask
                register_access_bits = access_bits(register_access)
                if register_access_bits & READ:
                    read_mask = implemented_mask
                if register_access_bits & WRITE:
                    write_mask = implemented_mask
            dimension = integer(child_text(register, "dim", "1"))
            increment = integer(child_text(register, "dimIncrement", "0"))
            dimension_indices = child_text(register, "dimIndex", "").split(",")
            for index in range(dimension):
                indexed_reset_value = reset_value
                if peripheral_name == "DMA" and name == "DCHPRI%s":
                    indexed_reset_value = integer(dimension_indices[index])
                if peripheral_name == "FTFA" and name == "FSTAT":
                    indexed_reset_value = 0x80
                registers.append(
                    Register(
                        base + offset + index * increment,
                        indexed_reset_value,
                        reset_mask,
                        implemented_mask,
                        read_mask,
                        write_mask,
                        0,
                        peripheral_index,
                        width,
                        (READ if read_mask else 0) | (WRITE if write_mask else 0),
                        peripheral_name,
                        name,
                    )
                )
    registers.sort(key=lambda item: (item.address, item.width))
    merged = []
    for register in registers:
        if not merged or (merged[-1].address, merged[-1].width) != (
            register.address,
            register.width,
        ):
            merged.append(register)
            continue
        previous = merged[-1]
        merged[-1] = Register(
            previous.address,
            previous.reset_value | register.reset_value,
            previous.reset_mask | register.reset_mask,
            previous.implemented_mask | register.implemented_mask,
            previous.read_mask | register.read_mask,
            previous.write_mask | register.write_mask,
            0,
            previous.peripheral_index,
            previous.width,
            previous.access | register.access,
            previous.peripheral,
            previous.name,
        )
    return peripherals, merged


def descriptor_values(path):
    source = Path(path).read_text(encoding="utf-8")
    pattern = re.compile(
        r"\{(0x[0-9a-f]+)u,\s*(0x[0-9a-f]+)u,\s*(0x[0-9a-f]+)u,\s*"
        r"(0x[0-9a-f]+)u,\s*(0x[0-9a-f]+)u,\s*(0x[0-9a-f]+)u,\s*"
        r"(0x[0-9a-f]+)u,\s*(\d+)u,\s*(\d+)u,\s*\(KinetisRegisterAccess\)(\d+)\}"
    )
    return {integer(match.group(1)): tuple(integer(value) for value in match.groups()[1:]) for match in pattern.finditer(source)}


def w1c_reference(svd_path, source_path):
    _, registers = flatten_svd(svd_path)
    values = descriptor_values(source_path)
    reference = {}
    for register in registers:
        descriptor = values.get(register.address)
        if descriptor is not None and descriptor[5] != 0:
            reference[(peripheral_kind(register.peripheral), register.name, register.width)] = descriptor[5]
    return reference


def with_w1c(register, mask):
    return Register(
        register.address,
        register.reset_value,
        register.reset_mask,
        register.implemented_mask,
        register.read_mask,
        register.write_mask,
        mask,
        register.peripheral_index,
        register.width,
        register.access,
        register.peripheral,
        register.name,
    )


def hash_byte(value, byte):
    return ((value ^ byte) * FNV_PRIME) & 0xFFFFFFFFFFFFFFFF


def hash_integer(value, integer_value, size):
    for byte in range(size):
        value = hash_byte(value, (integer_value >> (byte * 8)) & 0xFF)
    return value


def register_digest(registers):
    value = FNV_OFFSET
    for register in registers:
        for field, size in (
            (register.address, 4),
            (register.reset_value, 4),
            (register.reset_mask, 4),
            (register.implemented_mask, 4),
            (register.read_mask, 4),
            (register.write_mask, 4),
            (register.w1c_mask, 4),
            (register.peripheral_index, 2),
            (register.width, 1),
            (register.access, 1),
        ):
            value = hash_integer(value, field, size)
    return value


def peripheral_digest(peripherals):
    value = FNV_OFFSET
    for peripheral in peripherals:
        for byte in peripheral.encode("utf-8") + b"\0":
            value = hash_byte(value, byte)
    return value


def format_c(symbol, profile, peripherals, registers):
    lines = [
        '#include "device/kinetis/variants/register_data.h"',
        "",
        "#define COUNT(array) (sizeof(array) / sizeof((array)[0]))",
        "",
        f"static const char* const {symbol}_peripherals[] = {{",
    ]
    lines.extend(f'    "{name}",' for name in peripherals)
    lines.extend(["};", "", f"static const KinetisRegisterDescriptor {symbol}_registers[] = {{"])
    for register in registers:
        values = (
            register.address,
            register.reset_value,
            register.reset_mask,
            register.implemented_mask,
            register.read_mask,
            register.write_mask,
            register.w1c_mask,
        )
        prefix = "    {" + ", ".join(f"0x{value:08x}u" for value in values)
        lines.append(f"{prefix}, {register.peripheral_index}u,")
        lines.append(f"     {register.width}u, (KinetisRegisterAccess){register.access}}},")
    lines.extend(
        [
            "};",
            "",
            f"const KinetisRegisterManifest* kinetis_{symbol}_register_manifest(void) {{",
            "    static const KinetisRegisterManifest manifest = {",
            f"        {profile}, {symbol}_registers, COUNT({symbol}_registers),",
            f"        {symbol}_peripherals, COUNT({symbol}_peripherals), UINT64_C(0x{register_digest(registers):016x}),",
            f"        UINT64_C(0x{peripheral_digest(peripherals):016x}),",
            "    };",
            "    return &manifest;",
            "}",
            "",
        ]
    )
    return "\n".join(lines)


def format_register_def(registers):
    lines = []
    for register in registers:
        values = (
            register.address,
            register.reset_value,
            register.reset_mask,
            register.implemented_mask,
            register.read_mask,
            register.write_mask,
            register.w1c_mask,
        )
        prefix = "KINETIS_EXPECTED_REGISTER(" + ", ".join(f"0x{value:08x}u" for value in values)
        lines.append(
            f"{prefix}, {register.peripheral_index}u, {register.width}u, {register.access})"
        )
    return "\n".join(lines) + "\n"


def format_peripheral_def(peripherals):
    return "".join(f'KINETIS_EXPECTED_PERIPHERAL("{name}")\n' for name in peripherals)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("svd", type=Path)
    parser.add_argument("--symbol", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--registers", type=Path, required=True)
    parser.add_argument("--peripherals", type=Path, required=True)
    parser.add_argument("--reference-svd", type=Path)
    parser.add_argument("--reference-source", type=Path)
    arguments = parser.parse_args()
    peripherals, registers = flatten_svd(arguments.svd)
    reference = {}
    if arguments.reference_svd is not None and arguments.reference_source is not None:
        reference = w1c_reference(arguments.reference_svd, arguments.reference_source)
    registers = [
        with_w1c(
            register,
            reference.get((peripheral_kind(register.peripheral), register.name, register.width), 0)
            & register.write_mask,
        )
        for register in registers
    ]
    arguments.source.write_text(
        format_c(arguments.symbol, arguments.profile, peripherals, registers), encoding="utf-8"
    )
    arguments.registers.write_text(format_register_def(registers), encoding="utf-8")
    arguments.peripherals.write_text(format_peripheral_def(peripherals), encoding="utf-8")


if __name__ == "__main__":
    main()
