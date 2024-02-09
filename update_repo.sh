git add .
git rm -r --cached *.DS_Store
#git rm --cached *.sh
git rm -r --cached .gitignore
git rm -r --cached scalexi_api.egg-info
git rm -r --cached build/

git commit -m "commit changes v0.1.0"

# This sets a new tag for the latest commit
git tag v0.1.0

# This moves the v0.1.1 tag to where v0.1.0 was previously
# You might need to use -f flag to force the move if the tag already exists
#git tag -f v0.1.1 59fe1e4cc0162eb9e595e9ff5a974f8c8f86e159

# Push the latest commit to the main branch
git push origin main

# Push the new tags to the remote repository
git push origin v0.1.0
git push origin -f v0.1.0  # Force is required if the tag already exists and you're moving it