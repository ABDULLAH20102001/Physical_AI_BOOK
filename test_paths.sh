#!/bin/bash
# Test script to verify the fixed get_feature_paths function

# Source the fixed common functions - properly quoted path
source "/mnt/j/Quter4_prompt engineering+/Teaching Physical AI  Humanoid Robotics Course/.specify/scripts/bash/common_fixed.sh"

# Get feature paths
eval $(get_feature_paths)

# Display the paths to verify they're properly set
echo "REPO_ROOT: $REPO_ROOT"
echo "CURRENT_BRANCH: $CURRENT_BRANCH"
echo "FEATURE_DIR: $FEATURE_DIR"
echo "FEATURE_SPEC: $FEATURE_SPEC"
echo "IMPL_PLAN: $IMPL_PLAN"
echo "TASKS: $TASKS"

# Check if the feature directory exists
if [[ -d "$FEATURE_DIR" ]]; then
    echo "✅ Feature directory exists: $FEATURE_DIR"
else
    echo "❌ Feature directory does not exist: $FEATURE_DIR"
fi

# Check if the required files exist
if [[ -f "$FEATURE_SPEC" ]]; then
    echo "✅ Spec file exists: $FEATURE_SPEC"
else
    echo "❌ Spec file does not exist: $FEATURE_SPEC"
fi

if [[ -f "$IMPL_PLAN" ]]; then
    echo "✅ Plan file exists: $IMPL_PLAN"
else
    echo "❌ Plan file does not exist: $IMPL_PLAN"
fi

if [[ -f "$TASKS" ]]; then
    echo "✅ Tasks file exists: $TASKS"
else
    echo "❌ Tasks file does not exist: $TASKS"
fi