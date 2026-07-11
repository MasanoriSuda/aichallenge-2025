if(NOT DEFINED VALIDATOR OR NOT DEFINED CSV OR NOT DEFINED EXPECTED_EXIT OR
  NOT DEFINED EXPECTED_ERROR)
  message(FATAL_ERROR "validator contract test is missing a required argument")
endif()

execute_process(
  COMMAND "${VALIDATOR}" "${CSV}"
  RESULT_VARIABLE actual_exit
  OUTPUT_VARIABLE validator_stdout
  ERROR_VARIABLE validator_stderr
)

if(NOT "${actual_exit}" STREQUAL "${EXPECTED_EXIT}")
  message(FATAL_ERROR
    "expected validator exit ${EXPECTED_EXIT}, got ${actual_exit}\n"
    "stdout:\n${validator_stdout}\n"
    "stderr:\n${validator_stderr}")
endif()

set(validator_output "${validator_stdout}\n${validator_stderr}")
string(FIND "${validator_output}" "${EXPECTED_ERROR}" expected_error_index)
if(expected_error_index EQUAL -1)
  message(FATAL_ERROR
    "validator output did not contain '${EXPECTED_ERROR}'\n"
    "stdout:\n${validator_stdout}\n"
    "stderr:\n${validator_stderr}")
endif()
