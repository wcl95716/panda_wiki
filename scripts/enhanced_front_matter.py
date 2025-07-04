#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hugo文章Front Matter自动填充工具
自动为文章添加创建时间、修改时间、作者和分类信息
"""

import os
import sys
import re
import datetime
from pathlib import Path
import subprocess
from typing import Optional, Tuple, Dict, List

# 配置信息
CONFIG = {
    "author": "panda",  # 默认作者
    "content_dirs": [
        "panda-blog"
    ],
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

def get_file_times(filepath: Path) -> Tuple[Optional[datetime.datetime], Optional[datetime.datetime]]:
    """获取文件的创建时间和修改时间"""
    try:
        stat = filepath.stat()
        # macOS 下用 st_birthtime，Linux 下用 st_ctime
        if hasattr(stat, 'st_birthtime'):
            created_time = datetime.datetime.fromtimestamp(stat.st_birthtime)
        else:
            created_time = datetime.datetime.fromtimestamp(stat.st_ctime)
        modified_time = datetime.datetime.fromtimestamp(stat.st_mtime)
        return created_time, modified_time
    except Exception as e:
        print(f"❌ 获取文件时间失败: {e}")
        return None, None

def get_git_times(filepath: Path) -> Tuple[Optional[datetime.datetime], Optional[datetime.datetime]]:
    """从Git历史获取文件的创建时间和最后修改时间"""
    try:
        # 获取文件在Git中的创建时间（第一次提交）
        result = subprocess.run(
            ['git', 'log', '--follow', '--format=%aI', '--', str(filepath)],
            capture_output=True, text=True, cwd=filepath.parent.parent.parent
        )
        
        if result.returncode == 0 and result.stdout.strip():
            # 获取第一次提交时间（创建时间）
            first_commit = result.stdout.strip().split('\n')[-1]
            created_time = datetime.datetime.fromisoformat(first_commit.replace('Z', '+00:00'))
            
            # 获取最后一次提交时间（修改时间）
            last_commit = result.stdout.strip().split('\n')[0]
            modified_time = datetime.datetime.fromisoformat(last_commit.replace('Z', '+00:00'))
            
            return created_time, modified_time
    except Exception as e:
        print(f"⚠️  无法从Git获取时间: {e}")
    
    return None, None

def get_categories_from_path(filepath: Path) -> List[str]:
    """根据文件路径自动生成分类"""
    categories = []
    
    # 获取相对于项目根目录的路径
    try:
        rel_path = filepath.relative_to(Path.cwd())
        path_parts = list(rel_path.parts)
    except ValueError:
        # 如果无法获取相对路径，使用绝对路径的最后几部分
        path_parts = list(filepath.parts)[-3:]  # 取最后3级目录
    
    # 移除文件名
    path_parts = path_parts[:-1]
    
    for part in path_parts:
        if part in CONFIG["category_mapping"]:
            categories.append(CONFIG["category_mapping"][part])
        else:
            # 如果没有映射，直接使用目录名
            categories.append(part)
    
    return categories

def parse_front_matter(content: str) -> Tuple[dict, int, int]:
    """递归解析front matter，支持嵌套字典"""
    lines = content.split('\n')
    front_matter = {}
    start_line = -1
    end_line = -1
    
    def parse_block(start):
        d = {}
        i = start
        while i < len(lines):
            line = lines[i]
            if line.strip() == '---':
                return d, i
            if not line.strip() or line.strip().startswith('#'):
                i += 1
                continue
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                if value == '':
                    # 可能是嵌套dict
                    # 检查下一行是否缩进
                    sub_dict, next_i = parse_block(i+1)
                    d[key] = sub_dict
                    i = next_i
                    continue
                # 处理数组
                if value.startswith('[') and value.endswith(']'):
                    items = value[1:-1].split(',')
                    value = [item.strip().strip('"\'') for item in items if item.strip()]
                elif value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                d[key] = value
            i += 1
        return d, i
    
    if lines and lines[0].strip() == '---':
        start_line = 0
        d, end_line = parse_block(1)
        front_matter = d
    return front_matter, start_line, end_line

def format_date(dt: datetime.datetime) -> str:
    """格式化日期为Hugo格式"""
    # 确保时区信息正确格式化
    if dt.tzinfo is None:
        # 如果没有时区信息，使用本地时区
        import time
        local_offset = time.timezone if time.daylight == 0 else time.altzone
        dt = dt.replace(tzinfo=datetime.timezone(datetime.timedelta(seconds=-local_offset)))
    
    return dt.strftime('%Y-%m-%dT%H:%M:%S%z')

def generate_title_from_filename(filename: str) -> str:
    """从文件名生成标题"""
    # 移除.md扩展名
    title = filename.replace('.md', '')
    
    # 如果是中文文件名，直接使用
    if re.search(r'[\u4e00-\u9fff]', title):
        return title
    
    # 英文文件名转换为标题格式
    title = title.replace('-', ' ').replace('_', ' ')
    title = ' '.join(word.capitalize() for word in title.split())
    
    return title

def update_post_front_matter(filepath: Path, force_update: bool = False, 
                           update_dates_only: bool = False, update_lastmod_only: bool = False,
                           use_current_time: bool = False) -> bool:
    """更新单个文章的Front Matter"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        front_matter, start_line, end_line = parse_front_matter(content)
        
        # 检查是否需要更新
        has_date = 'date' in front_matter
        has_author = 'author' in front_matter
        has_categories = 'categories' in front_matter
        has_title = 'title' in front_matter
        has_lastmod = 'lastmod' in front_matter
        
        # 获取文件的最后修改时间
        file_created, file_modified = get_file_times(filepath)
        git_created, git_modified = get_git_times(filepath)
        
        # 优先使用Git时间，如果没有则使用文件时间
        created_time = git_created or file_created
        file_real_modified = git_modified or file_modified
        
        # 读取 front matter 里的 lastmod 字段
        existing_lastmod = None
        if has_lastmod:
            try:
                existing_lastmod = datetime.datetime.fromisoformat(front_matter['lastmod'].replace('Z', '+00:00'))
                if existing_lastmod.tzinfo is None:
                    existing_lastmod = existing_lastmod.replace(tzinfo=datetime.timezone.utc)
            except Exception:
                pass
        
        if file_real_modified and file_real_modified.tzinfo is None:
            file_real_modified = file_real_modified.replace(tzinfo=datetime.timezone.utc)
        
        # 比较文件的最后修改时间和 lastmod 是否相等
        if existing_lastmod and file_real_modified:
            # 将时间都转换为 UTC 时区进行比较
            if existing_lastmod.tzinfo is None:
                existing_lastmod = existing_lastmod.replace(tzinfo=datetime.timezone.utc)
            else:
                existing_lastmod = existing_lastmod.astimezone(datetime.timezone.utc)
            
            if file_real_modified.tzinfo is None:
                file_real_modified = file_real_modified.replace(tzinfo=datetime.timezone.utc)
            else:
                file_real_modified = file_real_modified.astimezone(datetime.timezone.utc)
            
            # 将时间转换为时间戳进行比较，精度到分钟
            lastmod_timestamp = int(existing_lastmod.timestamp() / 60) * 60
            mtime_timestamp = int(file_real_modified.timestamp() / 60) * 60
            if lastmod_timestamp == mtime_timestamp:
                print(f"⏭️  跳过 {filepath.name} (lastmod 与文件修改时间相等)")
                return False
        
        # 更新 front matter 字段
        if not has_date or force_update:
            if created_time:
                front_matter['date'] = format_date(created_time)
        if not has_author or force_update:
            front_matter['author'] = CONFIG["author"]
        if not has_categories or force_update:
            categories = get_categories_from_path(filepath)
            if categories:
                front_matter['categories'] = categories
        if not has_title or force_update:
            front_matter['title'] = generate_title_from_filename(filepath.name)
        
        # lastmod 跟随 mtime
        if file_real_modified:
            front_matter['lastmod'] = format_date(file_real_modified)
        
        # 重新生成 front matter 内容
        new_front_matter_lines = ['---']
        new_front_matter_lines.extend(dump_front_matter(front_matter))
        new_front_matter_lines.append('---')
        content_lines = content.split('\n')
        if start_line >= 0 and end_line >= 0:
            new_content = '\n'.join(new_front_matter_lines) + '\n' + '\n'.join(content_lines[end_line + 1:])
        else:
            new_content = '\n'.join(new_front_matter_lines) + '\n\n' + content
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        print(f"✅ 更新 {filepath.name}")
        if created_time:
            print(f"   📅 创建时间: {created_time.strftime('%Y-%m-%d %H:%M:%S')}")
        if file_real_modified:
            print(f"   🔄 修改时间: {file_real_modified.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"   👤 作者: {front_matter.get('author', 'N/A')}")
        print(f"   📂 分类: {', '.join(front_matter.get('categories', []))}")
        print(f"   📝 标题: {front_matter.get('title', 'N/A')}")
        
        return True
        
    except Exception as e:
        print(f"❌ 更新 {filepath.name} 失败: {e}")
        return False

def find_markdown_files(content_dirs: List[str]) -> List[Path]:
    """查找所有markdown文件"""
    markdown_files = []
    
    for content_dir in content_dirs:
        content_path = Path(content_dir)
        
        if not content_path.exists():
            print(f"⚠️  目录不存在: {content_dir}")
            continue
        
        for filepath in content_path.rglob("*.md"):
            if filepath.is_file():
                markdown_files.append(filepath)
    
    return markdown_files

def dump_front_matter(d, indent=0):
    """将字典转换为front matter格式"""
    lines = []
    for key, value in d.items():
        prefix = '  ' * indent
        if isinstance(value, list):
            items_str = ', '.join([f'"{item}"' for item in value])
            lines.append(f'{prefix}{key}: [{items_str}]')
        elif isinstance(value, dict):
            lines.append(f'{prefix}{key}:')
            lines.extend(dump_front_matter(value, indent+1))
        else:
            # 字符串加引号
            if isinstance(value, str) and (':' in value or value.strip() == '' or value.startswith('@')):
                value = f'"{value}"'
            lines.append(f'{prefix}{key}: {value}')
    return lines

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Hugo文章Front Matter自动填充工具")
    parser.add_argument("--content-dirs", nargs='+', default=CONFIG["content_dirs"], 
                       help="内容目录路径列表")
    parser.add_argument("--file", help="指定单个文件")
    parser.add_argument("--force", action="store_true", default=True, help="强制更新已有字段的文件（默认开启）")
    parser.add_argument("--dry-run", action="store_true", help="仅显示将要进行的更改，不实际修改文件")
    parser.add_argument("--author", default=CONFIG["author"], help="设置作者名称")
    parser.add_argument("--update-dates", action="store_true", help="仅更新日期相关字段")
    parser.add_argument("--update-lastmod", action="store_true", help="仅更新最后修改时间")
    parser.add_argument("--use-current-time", action="store_true", help="使用当前时间作为修改时间")
    
    args = parser.parse_args()
    
    # 更新配置
    CONFIG["author"] = args.author
    
    if args.file:
        # 处理单个文件
        filepath = Path(args.file)
        if not filepath.exists():
            print(f"❌ 文件不存在: {args.file}")
            sys.exit(1)
        
        if args.dry_run:
            print(f"🔍 预览模式 - {filepath}")
            created_time, modified_time = get_file_times(filepath)
            git_created, git_modified = get_git_times(filepath)
            
            # 优先使用Git时间
            final_created = git_created or created_time
            final_modified = git_modified or modified_time
            
            if args.use_current_time:
                final_modified = datetime.datetime.now()
                print(f"   🔄 修改时间: {final_modified.strftime('%Y-%m-%d %H:%M:%S')} (当前时间)")
            else:
                if final_created:
                    print(f"   创建时间: {final_created.strftime('%Y-%m-%d %H:%M:%S')}")
                if final_modified:
                    print(f"   修改时间: {final_modified.strftime('%Y-%m-%d %H:%M:%S')}")
            
            if not (args.update_dates or args.update_lastmod):
                print(f"   作者: {CONFIG['author']}")
                categories = get_categories_from_path(filepath)
                print(f"   分类: {', '.join(categories)}")
                print(f"   标题: {generate_title_from_filename(filepath.name)}")
        else:
            update_post_front_matter(filepath, args.force, args.update_dates, 
                                   args.update_lastmod, args.use_current_time)
    else:
        # 处理所有文件
        markdown_files = find_markdown_files(args.content_dirs)
        
        if not markdown_files:
            print(f"❌ 在指定目录中没有找到markdown文件")
            sys.exit(1)
        
        print(f"📁 找到 {len(markdown_files)} 个markdown文件")
        
        # 显示操作模式
        if args.update_dates:
            print("🔄 模式: 仅更新日期字段")
        elif args.update_lastmod:
            print("🔄 模式: 仅更新最后修改时间")
        elif args.use_current_time:
            print("🔄 模式: 使用当前时间作为修改时间")
        else:
            print(f"👤 作者: {CONFIG['author']}")
            print(f"📂 内容目录: {', '.join(args.content_dirs)}")
        
        if args.dry_run:
            print("🔍 预览模式 - 显示将要进行的更改:")
            for filepath in markdown_files:
                created_time, modified_time = get_file_times(filepath)
                git_created, git_modified = get_git_times(filepath)
                
                # 优先使用Git时间
                final_created = git_created or created_time
                final_modified = git_modified or modified_time
                
                if args.use_current_time:
                    final_modified = datetime.datetime.now()
                
                print(f"  {filepath}:")
                if args.update_dates or args.update_lastmod:
                    if final_created and (args.update_dates or not args.update_lastmod):
                        print(f"    创建时间: {final_created.strftime('%Y-%m-%d %H:%M:%S')}")
                    if final_modified:
                        print(f"    修改时间: {final_modified.strftime('%Y-%m-%d %H:%M:%S')}")
                else:
                    if final_created:
                        print(f"    创建时间: {final_created.strftime('%Y-%m-%d %H:%M:%S')}")
                    if final_modified:
                        print(f"    修改时间: {final_modified.strftime('%Y-%m-%d %H:%M:%S')}")
                    print(f"    作者: {CONFIG['author']}")
                    categories = get_categories_from_path(filepath)
                    print(f"    分类: {', '.join(categories)}")
                    title = generate_title_from_filename(filepath.name)
                    print(f"    标题: {title}")
        else:
            updated_count = 0
            for filepath in markdown_files:
                if update_post_front_matter(filepath, args.force, args.update_dates, 
                                          args.update_lastmod, args.use_current_time):
                    updated_count += 1
            
            print(f"\n📊 更新完成: {updated_count}/{len(markdown_files)} 个文件")

if __name__ == "__main__":
    main() 