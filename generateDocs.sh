#!/bin/bash

# Run Doxygen to generate documentation
doxygen Doxyfile

# Check if the documentation was generated successfully
if [ -d "doc/html" ]; then
  echo "Documentation generated successfully."
  # Open the index.html file in the default web browser
  open doc/html/index.html
else
  echo "Error generating documentation."
fi
