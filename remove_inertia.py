#!/usr/bin/env python3
import os
import re
from pathlib import Path

models_dir = Path.home() / "Internships/ProjectArion/project_arion/src/aws-robomaker-small-warehouse-world/models"

for model_dir in sorted(models_dir.glob("aws_robomaker_warehouse_*")):
    model_file = model_dir / "model.sdf"
    if model_file.exists():
        with open(model_file, 'r') as f:
            content = f.read()
        
        # Remove inertial blocks using regex
        content_cleaned = re.sub(r'<inertial>.*?</inertial>', '', content, flags=re.DOTALL)
        
        with open(model_file, 'w') as f:
            f.write(content_cleaned)
        
        print(f"Processed: {model_dir.name}")

print("All inertial blocks removed from all models!")
