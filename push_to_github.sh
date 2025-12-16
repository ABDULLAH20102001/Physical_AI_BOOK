#!/bin/bash

echo "==============================================="
echo "Pushing Teaching Physical AI Humanoid Robotics Course to GitHub"
echo "==============================================="

# Navigate to project directory
cd "$(dirname "$0")"

echo "Current branch:"
git branch

echo ""
echo "Current status:"
git status --porcelain

echo ""
echo "Remote repositories:"
git remote -v

echo ""
echo "Attempting to push to origin..."

# Store credentials temporarily if needed
git config credential.helper cache

# Try to push the current branch to origin
git push -u origin 001-physical-ai-humanoid-robotics

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Successfully pushed to GitHub!"
    echo ""
    echo "Your project has been uploaded to: https://github.com/ABDULLAH20102001/Hackathon-Physical-AI-.git"
    echo ""
else
    echo ""
    echo "❌ Push failed. You may need to:"
    echo "   1. Check your GitHub credentials"
    echo "   2. Generate a personal access token at GitHub.com"
    echo "   3. Run: git config --global credential.helper store"
    echo "   4. Then retry the push command"
    echo ""
fi