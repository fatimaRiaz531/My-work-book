# PowerShell script to push code to GitHub
cd E:\itcourse\hachathon\humanoid-robotic-book

Write-Host "Initializing git repository..."
git init

Write-Host "Adding remote..."
git remote remove origin 2>$null
git remote add origin https://github.com/fatimaRiaz531/My-work-book.git

Write-Host "Adding all files..."
git add .

Write-Host "Committing..."
git commit -m "Initial commit: Physical AI & Humanoid Robotics textbook with RAG chatbot, authentication, personalization, and Urdu translation"

Write-Host "Setting branch to main..."
git branch -M main

Write-Host "Pushing to GitHub..."
git push -u origin main --force

Write-Host "Done! Check https://github.com/fatimaRiaz531/My-work-book"

