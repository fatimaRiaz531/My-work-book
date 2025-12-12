# ⚡ Code GitHub Par Push Karne Ke Liye

Aapko manually yeh commands run karni hongi kyunki terminal output issue hai.

## Step-by-Step Instructions:

### Method 1: PowerShell/CMD Mein (Recommended)

1. **PowerShell ya CMD open karo** (Windows + R, type `powershell`)

2. **Yeh commands ek-ek karke run karo:**

```powershell
# 1. Folder mein jao
cd E:\itcourse\hachathon\humanoid-robotic-book

# 2. Git status check karo
git status

# 3. Sab files add karo
git add .

# 4. Commit karo
git commit -m "Complete hackathon project: RAG chatbot, auth, personalization, Urdu translation"

# 5. Remote set karo
git remote set-url origin https://github.com/fatimaRiaz531/My-work-book.git

# 6. Main branch set karo
git branch -M main

# 7. Push karo
git push -u origin main
```

### Method 2: VS Code Se (Easiest)

1. **VS Code open karo**
2. **File → Open Folder** → `E:\itcourse\hachathon\humanoid-robotic-book`
3. **Left side Source Control icon** (Ctrl+Shift+G) click karo
4. **Sab files ko stage karo** (files ke saamne + button)
5. **Top par commit message likho**: `"Complete hackathon project"`
6. **Commit** button click karo
7. **Sync Changes** ya **Push** button click karo

### Agar Authentication Error Aaye:

1. GitHub par jao: https://github.com/settings/tokens
2. **Generate new token (classic)** click karo
3. **repo** permission select karo
4. Token copy karo
5. Push karte waqt use karo:

```powershell
git push https://YOUR_TOKEN@github.com/fatimaRiaz531/My-work-book.git main
```

## Verify:

Push ke baad yeh URL check karo:
**https://github.com/fatimaRiaz531/My-work-book**

Code dikhna chahiye! ✅

