#!/bin/bash

echo "🚀 GitHub yükləmə prosesi avtomatik başlayır..."

# Layihə qovluğuna keçid
cd "/home/tural/Pluto +" || exit

# Git repozitoriyasını başlatmaq (əgər yoxdursa)
if [ ! -d ".git" ]; then
    git init
    echo "✅ Git repozitoriyası başladıldı."
fi

# Bütün faylları izləməyə əlavə et
git add .

# Dəyişiklikləri yaddaşa vur
git commit -m "Avtomatik yükləmə: Pluto SDR layihəsi kodu"

git branch -M main

# Əgər uzaq (remote) server yoxdursa, istifadəçidən URL soruş
if ! git remote | grep -q "origin"; then
    read -p "GitHub Repozitoriyanızın URL-ni daxil edin (məs: https://github.com/Tural/Pluto.git): " REPO_URL
    git remote add origin "$REPO_URL"
fi

echo "⏳ Kodlar GitHub-a göndərilir..."
git push -u origin main
echo "🎉 Proses uğurla tamamlandı!"