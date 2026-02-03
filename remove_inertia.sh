#!/bin/bash
cd ~/Internships/ProjectArion/project_arion/src/aws-robomaker-small-warehouse-world/models

for model_dir in */; do
    model_file="$model_dir/model.sdf"
    if [ -f "$model_file" ]; then
        sed -i '/<inertial>/,/<\/inertial>/d' "$model_file"
        echo "Processed: $model_dir"
    fi
done

echo "All inertial blocks removed!"
