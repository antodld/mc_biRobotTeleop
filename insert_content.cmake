function(insert_content INPUT_TEMPLATE INPUT_FILE MARKER)

    message("HERE IS MAIN FILE ${INPUT_TEMPLATE} ")
    message("HERE IS INPUT_FILE ${INPUT_FILE} ")
    message("HERE IS MARKER ${MARKER} ")

    file(READ ${INPUT_TEMPLATE} TEMPLATE_CONTENT)
    file(READ ${INPUT_FILE} INSERT_CONTENT)

    # Perform the insertion after the specified marker.
    string(REGEX REPLACE "@${MARKER}" "${INSERT_CONTENT}" MODIFIED_CONTENT "${TEMPLATE_CONTENT}")
    file(WRITE ${INPUT_TEMPLATE} "${MODIFIED_CONTENT}")

endfunction()