# 上传工程文件夹

如果你想把本地的**整个工程文件夹**上传到这个仓库，推荐用 Git 命令：

```bash
# 1) 进入你的工程目录
cd /path/to/your-project

# 2) 初始化仓库（如果还没初始化）
git init

# 3) 关联远程仓库
git remote add origin https://github.com/LKY-LONTANO/-.git

# 4) 提交并推送
git add .
git commit -m "upload project folder"
git branch -M main
git push -u origin main
```

> 如果页面上传只看到“文件上传”，那是网页端限制。上传完整文件夹时，使用上面的 Git 方式最稳定。
