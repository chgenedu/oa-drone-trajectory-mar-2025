# This script runs tests/camera_utils_test.py from the main project directory
#
# This script is a workaround for people who run into issues
# with importing tests/common.py when executing camera_utils_test.py

with open("tests/camera_utils_test.py") as file:
    exec(file.read())
