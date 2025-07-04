#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hugo文章创建工具
使用方法: python create_post.py "文章标题" [分类] [标签1,标签2,...]
"""

import os
import sys
import datetime
from pathlib import Path

def create_post(title, category="技术教程", tags=None):
    """创建新的Hugo文章"""
    
    # 生成文件名（将中文和特殊字符转换为英文）
    filename = title.replace(" ", "-").replace("：", "-").replace(":", "-")
    filename = "".join(c for c in filename if c.isalnum() or c in "-_")
    filename = filename.lower()
    
    # 添加日期前缀
    today = datetime.datetime.now().strftime("%Y-%m-%d")
    filename = f"{today}-{filename}.md"
    
    # 确定文件路径
    if category:
        filepath = Path(f"content/posts/{category}/{filename}")
        filepath.parent.mkdir(parents=True, exist_ok=True)
    else:
        filepath = Path(f"content/posts/{filename}")
    
    # 生成front matter
    front_matter = f"""---
title: "{title}"
draft: false
description: ""
tags: {tags or []}
categories: {[category] if category else []}
keywords: []
author: "Admin"
showToc: true
TocOpen: true
---

# {title}

## 引言

在这里开始你的文章内容...

## 主要内容

### 小节标题

内容...

## 总结

文章总结...
"""
    
    # 写入文件
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(front_matter)
    
    print(f"✅ 文章已创建: {filepath}")
    print(f"📝 标题: {title}")
    print(f"📁 分类: {category}")
    print(f"🏷️  标签: {tags or '无'}")
    print(f"⏰ 日期: 将使用文件修改时间")
    
    return filepath

def main():
    if len(sys.argv) < 2:
        print("使用方法: python create_post.py '文章标题' [分类] [标签1,标签2,...]")
        print("示例: python create_post.py 'Docker容器化部署指南' '技术教程' 'Docker,容器化,部署'")
        sys.exit(1)
    
    title = sys.argv[1]
    category = sys.argv[2] if len(sys.argv) > 2 else "技术教程"
    tags = sys.argv[3].split(",") if len(sys.argv) > 3 else None
    
    create_post(title, category, tags)

if __name__ == "__main__":
    main() 