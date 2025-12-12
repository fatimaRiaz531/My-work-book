@echo off
cd /d E:\itcourse\hachathon\humanoid-robotic-book
echo Current directory: %CD%
echo.
echo Checking git status...
git status
echo.
echo Adding all files...
git add -A
echo.
echo Committing changes...
git commit -m "Update: Complete hackathon project with all features"
echo.
echo Setting remote...
git remote set-url origin https://github.com/fatimaRiaz531/My-work-book.git
echo.
echo Setting branch to main...
git branch -M main
echo.
echo Pushing to GitHub...
git push -u origin main
echo.
echo Done! Check https://github.com/fatimaRiaz531/My-work-book
pause

