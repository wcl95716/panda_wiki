#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hugo文章日期自动填充工具
自动为文章添加创建时间和修改时间到front matter中
"""

import os
import sys
import re
import datetime
from pathlib import Path
import subprocess
from typing import Optional, Tuple

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

def update_post_dates(filepath: Path, force_update: bool = False) -> bool:
    """更新单个文章的日期"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        front_matter, start_line, end_line = parse_front_matter(content)
        
        # 检查是否已经有date字段且不是强制更新
        if 'date' in front_matter and not force_update:
            print(f"⏭️  跳过 {filepath.name} (已有date字段)")
            return False
        
        # 获取时间信息
        file_created, file_modified = get_file_times(filepath)
        git_created, git_modified = get_git_times(filepath)
        
        # 优先使用Git时间，如果没有则使用文件时间
        created_time = git_created or file_created
        modified_time = git_modified or file_modified
        
        if not created_time:
            print(f"❌ 无法获取 {filepath.name} 的时间信息")
            return False
        
        # 更新front matter
        # date字段使用创建时间
        front_matter['date'] = format_date(created_time)
        # lastmod字段使用修改时间（如果与创建时间不同）
        if modified_time and modified_time != created_time:
            front_matter['lastmod'] = format_date(modified_time)
        elif modified_time:
            # 如果修改时间与创建时间相同，也设置lastmod
            front_matter['lastmod'] = format_date(modified_time)
        
        # 重新生成front matter内容
        new_front_matter_lines = ['---']
        new_front_matter_lines.extend(dump_front_matter(front_matter))
        new_front_matter_lines.append('---')
        
        # 重建文件内容
        content_lines = content.split('\n')
        if start_line >= 0 and end_line >= 0:
            # 有front matter，替换它
            new_content = '\n'.join(new_front_matter_lines) + '\n' + '\n'.join(content_lines[end_line + 1:])
        else:
            # 没有front matter，添加一个
            new_content = '\n'.join(new_front_matter_lines) + '\n\n' + content
        
        # 写回文件
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        print(f"✅ 更新 {filepath.name}")
        print(f"   📅 创建时间: {created_time.strftime('%Y-%m-%d %H:%M:%S')}")
        if modified_time and modified_time != created_time:
            print(f"   🔄 修改时间: {modified_time.strftime('%Y-%m-%d %H:%M:%S')}")
        
        return True
        
    except Exception as e:
        print(f"❌ 更新 {filepath.name} 失败: {e}")
        return False

def find_markdown_files(content_dir: str = "content") -> list:
    """查找所有markdown文件"""
    markdown_files = []
    content_path = Path(content_dir)
    
    if not content_path.exists():
        print(f"❌ 目录不存在: {content_dir}")
        return markdown_files
    
    for filepath in content_path.rglob("*.md"):
        if filepath.is_file():
            markdown_files.append(filepath)
    
    return markdown_files

def dump_front_matter(d, indent=0):
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
    
    parser = argparse.ArgumentParser(description="Hugo文章日期自动填充工具")
    parser.add_argument("--content-dir", default="content/posts", help="内容目录路径")
    parser.add_argument("--file", help="指定单个文件")
    parser.add_argument("--force", action="store_true", help="强制更新已有date字段的文件")
    parser.add_argument("--dry-run", action="store_true", help="仅显示将要进行的更改，不实际修改文件")
    
    args = parser.parse_args()
    
    if args.file:
        # 处理单个文件
        filepath = Path(args.file)
        if not filepath.exists():
            print(f"❌ 文件不存在: {args.file}")
            sys.exit(1)
        
        if args.dry_run:
            print(f"🔍 预览模式 - {filepath}")
            created_time, modified_time = get_file_times(filepath)
            if created_time:
                print(f"   创建时间: {created_time.strftime('%Y-%m-%d %H:%M:%S')}")
            if modified_time:
                print(f"   修改时间: {modified_time.strftime('%Y-%m-%d %H:%M:%S')}")
        else:
            update_post_dates(filepath, args.force)
    else:
        # 处理所有文件
        markdown_files = find_markdown_files(args.content_dir)
        
        if not markdown_files:
            print(f"❌ 在 {args.content_dir} 中没有找到markdown文件")
            sys.exit(1)
        
        print(f"📁 找到 {len(markdown_files)} 个markdown文件")
        
        if args.dry_run:
            print("🔍 预览模式 - 显示将要进行的更改:")
            for filepath in markdown_files:
                created_time, modified_time = get_file_times(filepath)
                if created_time:
                    print(f"  {filepath}: {created_time.strftime('%Y-%m-%d %H:%M:%S')}")
        else:
            updated_count = 0
            for filepath in markdown_files:
                if update_post_dates(filepath, args.force):
                    updated_count += 1
            
            print(f"\n📊 更新完成: {updated_count}/{len(markdown_files)} 个文件")

if __name__ == "__main__":
    main() 