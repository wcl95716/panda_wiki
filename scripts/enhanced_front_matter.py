#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hugo文章Front Matter自动填充工具
自动为文章添加创建时间、作者和分类信息
"""

import os
import re
import datetime
from pathlib import Path
import subprocess
from typing import Optional, Tuple, List

# 配置信息
CONFIG = {
    "author": "panda",
    "content_dirs": ["panda-blog"],
    "category_mapping": {
        "panda-blog": "技术文档",
        "想法": "创意想法",
        "vpn": "VPN技术",
        "ACM": "算法竞赛",
        "macos": "macOS",
        "server": "服务器",
        "工具": "工具使用"
    }
}

def get_git_times(filepath: Path) -> Tuple[Optional[datetime.datetime], Optional[datetime.datetime]]:
    """从Git历史获取文件的创建时间和最后修改时间"""
    try:
        result = subprocess.run(
            ['git', 'log', '--follow', '--format=%aI', '--', str(filepath)],
            capture_output=True, text=True, cwd=filepath.parent.parent.parent
        )
        
        if result.returncode == 0 and result.stdout.strip():
            first_commit = result.stdout.strip().split('\n')[-1]
            last_commit = result.stdout.strip().split('\n')[0]
            created_time = datetime.datetime.fromisoformat(first_commit.replace('Z', '+00:00'))
            modified_time = datetime.datetime.fromisoformat(last_commit.replace('Z', '+00:00'))
            return created_time, modified_time
    except Exception:
        pass
    return None, None

def get_categories_from_path(filepath: Path) -> List[str]:
    """根据文件路径自动生成分类"""
    try:
        rel_path = filepath.relative_to(Path.cwd())
        path_parts = list(rel_path.parts)[:-1]  # 移除文件名
    except ValueError:
        path_parts = list(filepath.parts)[-3:-1]  # 取最后2级目录
    
    categories = []
    for part in path_parts:
        if part in CONFIG["category_mapping"]:
            categories.append(CONFIG["category_mapping"][part])
        else:
            categories.append(part)
    return categories

def parse_front_matter(content: str) -> Tuple[dict, int, int]:
    """解析front matter"""
    lines = content.split('\n')
    front_matter = {}
    start_line = -1
    end_line = -1
    
    if lines and lines[0].strip() == '---':
        start_line = 0
        i = 1
        while i < len(lines):
            line = lines[i]
            if line.strip() == '---':
                end_line = i
                break
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                if value.startswith('[') and value.endswith(']'):
                    items = value[1:-1].split(',')
                    value = [item.strip().strip('"\'') for item in items if item.strip()]
                elif value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                front_matter[key] = value
            i += 1
    
    return front_matter, start_line, end_line

def format_date(dt: datetime.datetime) -> str:
    """格式化日期为Hugo格式"""
    if dt.tzinfo is None:
        import time
        local_offset = time.timezone if time.daylight == 0 else time.altzone
        dt = dt.replace(tzinfo=datetime.timezone(datetime.timedelta(seconds=-local_offset)))
    return dt.strftime('%Y-%m-%dT%H:%M:%S%z')

def generate_title_from_filename(filename: str) -> str:
    """从文件名生成标题"""
    title = filename.replace('.md', '')
    if re.search(r'[\u4e00-\u9fff]', title):
        return title
    title = title.replace('-', ' ').replace('_', ' ')
    return ' '.join(word.capitalize() for word in title.split())

def update_post_front_matter(filepath: Path) -> bool:
    """更新单个文章的Front Matter"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        front_matter, start_line, end_line = parse_front_matter(content)
        
        # 获取Git时间
        git_created, git_modified = get_git_times(filepath)
        created_time = git_created
        
        # 更新字段
        if not 'date' in front_matter and created_time:
            front_matter['date'] = format_date(created_time)
        if not 'author' in front_matter:
            front_matter['author'] = CONFIG["author"]
        if not 'categories' in front_matter:
            categories = get_categories_from_path(filepath)
            if categories:
                front_matter['categories'] = categories
        if not 'title' in front_matter:
            front_matter['title'] = generate_title_from_filename(filepath.name)
        
        # 生成新的front matter
        new_lines = ['---']
        for key, value in front_matter.items():
            if isinstance(value, list):
                items_str = ', '.join([f'"{item}"' for item in value])
                new_lines.append(f'{key}: [{items_str}]')
            else:
                if isinstance(value, str) and (':' in value or value.strip() == ''):
                    value = f'"{value}"'
                new_lines.append(f'{key}: {value}')
        new_lines.append('---')
        
        # 重建文件内容
        content_lines = content.split('\n')
        if start_line >= 0 and end_line >= 0:
            new_content = '\n'.join(new_lines) + '\n' + '\n'.join(content_lines[end_line + 1:])
        else:
            new_content = '\n'.join(new_lines) + '\n\n' + content
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        print(f"✅ 更新 {filepath.name}")
        return True
        
    except Exception as e:
        print(f"❌ 更新 {filepath.name} 失败: {e}")
        return False

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Hugo文章Front Matter自动填充工具")
    parser.add_argument("--content-dirs", nargs='+', default=CONFIG["content_dirs"], 
                       help="内容目录路径列表")
    parser.add_argument("--file", help="指定单个文件")
    parser.add_argument("--author", default=CONFIG["author"], help="设置作者名称")
    
    args = parser.parse_args()
    CONFIG["author"] = args.author
    
    if args.file:
        filepath = Path(args.file)
        if not filepath.exists():
            print(f"❌ 文件不存在: {args.file}")
            return
        update_post_front_matter(filepath)
    else:
        # 处理所有文件
        markdown_files = []
        for content_dir in args.content_dirs:
            content_path = Path(content_dir)
            if content_path.exists():
                markdown_files.extend(content_path.rglob("*.md"))
        
        if not markdown_files:
            print(f"❌ 在指定目录中没有找到markdown文件")
            return
        
        print(f"📁 找到 {len(markdown_files)} 个markdown文件")
        print(f"👤 作者: {CONFIG['author']}")
        
        updated_count = 0
        for filepath in markdown_files:
            if filepath.is_file() and update_post_front_matter(filepath):
                updated_count += 1
        
        print(f"\n📊 更新完成: {updated_count}/{len(markdown_files)} 个文件")

if __name__ == "__main__":
    main() 