#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hugo文章Front Matter自动填充工具
自动为文章添加创建时间、作者、分类信息和最后修改时间
"""

import os
import re
import datetime
import argparse
from pathlib import Path
import subprocess
from typing import Optional, Tuple, List

# 配置信息
CONFIG = {
    "author": "Panda",
    "content_dirs": [ "content/posts"],
    "category_mapping": {
        "想法": "创意想法",
        "vpn": "VPN技术",
        "acm": "算法竞赛",
        "macos": "macOS",
        "server": "服务器",
        "工具": "工具使用",
        "ros2": "ROS2 技术文档"
    }
}

# ==================== 命令行参数解析 ====================

def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='Hugo Front Matter 自动填充工具')
    parser.add_argument('--force', action='store_true', 
                       help='强制更新所有字段，包括已存在的字段')
    parser.add_argument('--force-git', action='store_true',
                       help='强制使用Git历史时间，忽略文件系统时间')
    parser.add_argument('--dry-run', action='store_true',
                       help='只显示将要进行的更改，不实际修改文件')
    return parser.parse_args()

# ==================== 文件处理函数 ====================

def get_git_creation_time(file_path: Path) -> Optional[datetime.datetime]:
    """从Git历史获取文件的首次提交时间（最准确的创建时间）"""
    try:
        # 检查文件是否在子仓库中
        current_dir = file_path.parent
        git_dir = None
        
        # 向上查找.git目录
        while current_dir != current_dir.parent:
            if (current_dir / '.git').exists():
                git_dir = current_dir
                break
            current_dir = current_dir.parent
        
        # 如果没找到，尝试从当前工作目录开始查找
        if not git_dir:
            current_dir = Path.cwd()
            while current_dir != current_dir.parent:
                if (current_dir / '.git').exists():
                    git_dir = current_dir
                    break
                current_dir = current_dir.parent
        
        if not git_dir:
            return None
        
        # 检查是否是浅克隆
        try:
            result = subprocess.run(
                ['git', 'rev-parse', '--is-shallow-repository'],
                capture_output=True, text=True, cwd=git_dir
            )
            is_shallow = result.returncode == 0 and result.stdout.strip() == 'true'
        except Exception:
            is_shallow = False
        
        # 计算相对于Git目录的路径
        try:
            relative_path = file_path.relative_to(git_dir)
        except ValueError:
            # 如果文件不在Git目录内，尝试使用绝对路径
            relative_path = file_path
        
        # 获取文件的首次提交时间
        result = subprocess.run(
            ['git', 'log', '--follow', '--format=%aI', '--', str(relative_path)],
            capture_output=True, text=True, cwd=git_dir
        )
        
        if result.returncode == 0 and result.stdout.strip():
            # 获取最后一行（最早的提交）
            lines = result.stdout.strip().split('\n')
            if lines:
                # 解析ISO 8601格式的时间
                time_str = lines[-1].strip()
                return datetime.datetime.fromisoformat(time_str.replace('Z', '+00:00'))
            
    except Exception as e:
        print(f"警告: 无法从Git获取创建时间 {file_path}: {e}")
    
    return None

def get_git_last_modified_time(file_path: Path) -> Optional[datetime.datetime]:
    """从Git历史获取文件的最后修改时间"""
    try:
        # 检查文件是否在子仓库中
        current_dir = file_path.parent
        git_dir = None
        
        # 向上查找.git目录
        while current_dir != current_dir.parent:
            if (current_dir / '.git').exists():
                git_dir = current_dir
                break
            current_dir = current_dir.parent
        
        # 如果没找到，尝试从当前工作目录开始查找
        if not git_dir:
            current_dir = Path.cwd()
            while current_dir != current_dir.parent:
                if (current_dir / '.git').exists():
                    git_dir = current_dir
                    break
                current_dir = current_dir.parent
        
        if not git_dir:
            return None
        
        # 计算相对于Git目录的路径
        try:
            relative_path = file_path.relative_to(git_dir)
        except ValueError:
            # 如果文件不在Git目录内，尝试使用绝对路径
            relative_path = file_path
        
        # 获取文件的最后提交时间
        result = subprocess.run(
            ['git', 'log', '-1', '--format=%aI', '--', str(relative_path)],
            capture_output=True, text=True, cwd=git_dir
        )
        
        if result.returncode == 0 and result.stdout.strip():
            time_str = result.stdout.strip()
            return datetime.datetime.fromisoformat(time_str.replace('Z', '+00:00'))
    except Exception as e:
        print(f"警告: 无法从Git获取修改时间 {file_path}: {e}")
    
    return None

def get_file_creation_time(file_path: Path, force_git: bool = False) -> datetime.datetime:
    """获取文件的创建时间，优先使用Git历史"""
    # 如果强制使用Git或Git时间可用，优先使用Git
    git_time = get_git_creation_time(file_path)
    if git_time and (force_git or git_time):
        return git_time
    
    # 如果Git获取失败，使用文件系统时间作为备选
    try:
        stat = file_path.stat()
        if hasattr(stat, 'st_birthtime'):
            return datetime.datetime.fromtimestamp(stat.st_birthtime)
        else:
            return datetime.datetime.fromtimestamp(stat.st_ctime)
    except Exception:
        return datetime.datetime.now()

def get_file_modification_time(file_path: Path, force_git: bool = False) -> datetime.datetime:
    """获取文件的最后修改时间，优先使用Git历史"""
    # 如果强制使用Git或Git时间可用，优先使用Git
    git_time = get_git_last_modified_time(file_path)
    if git_time and (force_git or git_time):
        return git_time
    
    # 如果Git获取失败，使用文件系统时间作为备选
    try:
        stat = file_path.stat()
        return datetime.datetime.fromtimestamp(stat.st_mtime)
    except Exception:
        return datetime.datetime.now()

def get_category_from_path(file_path: Path) -> str:
    """从文件路径推断分类"""
    # 获取文件的父目录名作为分类
    parent_dir = file_path.parent.name
    
    # 使用配置中的映射
    if parent_dir in CONFIG["category_mapping"]:
        return CONFIG["category_mapping"][parent_dir]
    
    # 如果没有映射，直接使用目录名
    return parent_dir

# ==================== Front Matter 处理函数 ====================

def parse_front_matter(content: str) -> Tuple[dict, str]:
    """解析Front Matter，返回元数据和内容"""
    # 匹配YAML Front Matter
    pattern = r'^---\s*\n(.*?)\n---\s*\n(.*)$'
    match = re.match(pattern, content, re.DOTALL)
    
    if match:
        yaml_content = match.group(1)
        markdown_content = match.group(2)
        
        # 改进的YAML解析，支持列表
        metadata = {}
        current_key = None
        current_list = []
        
        for line in yaml_content.split('\n'):
            line = line.strip()
            if not line:
                continue
                
            # 检查是否是列表项
            if line.startswith('- ') and current_key:
                current_list.append(line[2:].strip())
                continue
            elif current_list:
                # 结束当前列表
                metadata[current_key] = current_list
                current_list = []
                current_key = None
            
            # 检查是否是键值对
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip().strip('"\'')
                
                # 检查值是否为空（可能是列表的开始）
                if not value:
                    current_key = key
                    current_list = []
                else:
                    metadata[key] = value
        
        # 处理最后的列表
        if current_list:
            metadata[current_key] = current_list
        
        return metadata, markdown_content
    else:
        return {}, content

def generate_front_matter(metadata: dict, file_path: Path, force: bool = False, force_git: bool = False) -> str:
    """生成Front Matter"""
    # 获取文件创建时间和修改时间
    creation_time = get_file_creation_time(file_path, force_git)
    modification_time = get_file_modification_time(file_path, force_git)
    
    # 根据force参数决定是否更新已存在的字段
    if force or 'title' not in metadata:
        metadata['title'] = file_path.stem
    if force or 'date' not in metadata:
        # 生成ISO 8601格式的日期，包含时区信息
        metadata['date'] = f'"{creation_time.strftime("%Y-%m-%dT%H:%M:%S")}+0800"'
    if force or 'lastmod' not in metadata:
        # 添加最后修改时间
        metadata['lastmod'] = f'"{modification_time.strftime("%Y-%m-%dT%H:%M:%S")}+0800"'
    if force or 'author' not in metadata:
        metadata['author'] = CONFIG["author"]
    if force or 'categories' not in metadata:
        metadata['categories'] = [get_category_from_path(file_path)]
    
    # 生成YAML格式的Front Matter
    yaml_lines = ['---']
    for key, value in metadata.items():
        if isinstance(value, list):
            yaml_lines.append(f'{key}:')
            for item in value:
                yaml_lines.append(f'  - {item}')
        else:
            yaml_lines.append(f'{key}: {value}')
    yaml_lines.append('---')
    
    return '\n'.join(yaml_lines)

def update_file_front_matter(file_path: Path, force: bool = False, force_git: bool = False, dry_run: bool = False) -> bool:
    """更新单个文件的Front Matter"""
    try:
        # 读取文件内容
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 解析现有Front Matter
        metadata, markdown_content = parse_front_matter(content)
        
        # 获取时间信息并显示来源
        creation_time = get_file_creation_time(file_path, force_git)
        modification_time = get_file_modification_time(file_path, force_git)
        
        # 检查时间来源
        git_creation = get_git_creation_time(file_path)
        git_modification = get_git_last_modified_time(file_path)
        
        time_source = "Git历史" if git_creation else "文件系统"
        mod_source = "Git历史" if git_modification else "文件系统"
        
        # 生成新的Front Matter
        new_front_matter = generate_front_matter(metadata, file_path, force, force_git)
        
        # 组合新内容
        new_content = f"{new_front_matter}\n{markdown_content}"
        
        if dry_run:
            print(f"📝 预览更新: {file_path}")
            print(f"  创建时间: {creation_time.strftime('%Y-%m-%d %H:%M:%S')} (来源: {time_source})")
            print(f"  修改时间: {modification_time.strftime('%Y-%m-%d %H:%M:%S')} (来源: {mod_source})")
            print(f"  新的Front Matter:")
            print(f"  {new_front_matter.replace(chr(10), chr(10) + '  ')}")
            return True
        else:
            # 写回文件
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(new_content)
            
            print(f"✓ 已更新: {file_path}")
            print(f"  创建时间: {creation_time.strftime('%Y-%m-%d %H:%M:%S')} (来源: {time_source})")
            print(f"  修改时间: {modification_time.strftime('%Y-%m-%d %H:%M:%S')} (来源: {mod_source})")
            return True
        
    except Exception as e:
        print(f"✗ 更新失败 {file_path}: {e}")
        return False

# ==================== 主处理函数 ====================

def scan_and_update_files(force: bool = False, force_git: bool = False, dry_run: bool = False):
    """扫描并更新所有文件"""
    total_files = 0
    updated_files = 0
    
    for content_dir in CONFIG["content_dirs"]:
        dir_path = Path(content_dir)
        if not dir_path.exists():
            print(f"警告: 目录不存在 {content_dir}")
            continue
        
        # 扫描所有.md文件
        for md_file in dir_path.rglob("*.md"):
            total_files += 1
            if update_file_front_matter(md_file, force, force_git, dry_run):
                updated_files += 1
    
    mode = "预览" if dry_run else "更新"
    print(f"\n处理完成: {updated_files}/{total_files} 个文件已{mode}")

# ==================== 主程序入口 ====================

if __name__ == "__main__":
    args = parse_arguments()
    
    print("Hugo Front Matter 自动填充工具")
    print("=" * 40)
    
    if args.force_git:
        print("🔧 强制使用Git历史时间")
    if args.force:
        print("🔧 强制更新所有字段")
    if args.dry_run:
        print("🔧 预览模式 - 不会修改文件")
    
    print()
    scan_and_update_files(args.force, args.force_git, args.dry_run)
