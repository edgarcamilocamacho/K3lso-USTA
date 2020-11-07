#!/bin/bash

# Axis
sed -i 's/xyz=\"-1 0 0\"/xyz=\"1 0 0\"/g' k3lso_description.urdf
sed -i 's/xyz=\"0 -1 0\"/xyz=\"0 1 0\"/g' k3lso_description.urdf

# Color
sed -i 's/rgba=\".*\"/rgba=\"0.75 0.75 0.75 1.0\"/g' k3lso_description.urdf
