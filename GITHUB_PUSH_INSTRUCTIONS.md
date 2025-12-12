# GitHub Push Instructions

Agar code GitHub par show nahi ho raha, yeh steps follow karein:

## Method 1: Command Line se Push

```powershell
# 1. Repository folder mein jao
cd E:\itcourse\hachathon\humanoid-robotic-book

# 2. Git initialize karo (agar nahi hua ho)
git init

# 3. Remote add karo
git remote add origin https://github.com/fatimaRiaz531/My-work-book.git
# Ya agar already hai to update karo:
git remote set-url origin https://github.com/fatimaRiaz531/My-work-book.git

# 4. Sab files add karo
git add .

# 5. Commit karo
git commit -m "Initial commit: Physical AI & Humanoid Robotics textbook"

# 6. Main branch set karo
git branch -M main

# 7. Push karo (force push agar pehle se kuch hai)
git push -u origin main --force
```

## Method 2: GitHub Desktop se

1. GitHub Desktop install karo: https://desktop.github.com/
2. File → Add Local Repository
3. Folder select karo: `E:\itcourse\hachathon\humanoid-robotic-book`
4. Publish repository button click karo
5. Repository name: `My-work-book`
6. Publish karo

## Method 3: VS Code se

1. VS Code mein folder open karo
2. Source Control tab (Ctrl+Shift+G)
3. "Initialize Repository" click karo
4. Sab files stage karo (click on +)
5. Commit message likho aur commit karo
6. "Publish Branch" button click karo
7. GitHub par repository create karo

## Important Notes

- Agar authentication error aaye, to GitHub Personal Access Token use karo
- Token generate karo: https://github.com/settings/tokens
- Token se push karo: `git push https://YOUR_TOKEN@github.com/fatimaRiaz531/My-work-book.git`

## Verify

Push ke baad check karo:
https://github.com/fatimaRiaz531/My-work-book

Code dikhna chahiye!

