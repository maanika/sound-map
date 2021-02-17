# Refer to the instructions for using PSoC Creator with version control:
# https://community.cypress.com/docs/DOC-11030

# Add the PSOC Creator Files
git add *.c *.h *.cydwr *.cyprj .\TopDesign\*.cysch

# Add additional files
git add *.md

# Added these files
echo 'Git Status:'
git status

# Ask for the commit message
$CommitMsg = Read-Host -Prompt 'Enter the commit message'

# Commit
git commit --m "$CommitMsg"

# Push
git push -u origin main

# Keep the console open after execution
Read-Host -Prompt 'Press Enter to exit'