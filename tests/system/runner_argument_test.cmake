function(expect_runner_failure expected_error)
  execute_process(
    COMMAND "${RUNNER}" ${ARGN}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)
  if(result EQUAL 0 OR NOT error MATCHES "${expected_error}")
    message(FATAL_ERROR "unexpected runner result for ${ARGN}: ${output}${error}")
  endif()
endfunction()

expect_runner_failure("^usage: ")
expect_runner_failure("^usage: " missing.bin --reset-address)
expect_runner_failure("^usage: " missing.bin --max-instructions 1)
expect_runner_failure("^unknown K22 profile: invalid" missing.bin --reset-address 1 --profile invalid)
expect_runner_failure("^unknown K22 package: invalid" missing.bin --reset-address 1 --package invalid)
expect_runner_failure("^invalid value: invalid" missing.bin --reset-address invalid)
expect_runner_failure("^unknown option: --invalid" missing.bin --reset-address 1 --invalid 1)
expect_runner_failure("^an address is too large" missing.bin --reset-address 4294967296)
expect_runner_failure(
  "^failed to create the device"
  missing.bin
  --reset-address 1
  --profile MK22FN51212
  --package AK)
expect_runner_failure(
  "^failed to load the firmware image"
  missing.bin
  --reset-address 1
  --profile MK22FN51212
  --package DC
  --binary-address 0
  --max-cycles 1
  --stop-address 2)
