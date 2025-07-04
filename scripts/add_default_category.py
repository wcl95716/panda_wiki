import os
import re

def add_default_category_to_md(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 检查是否有 front matter
    if not content.startswith('---'):
        return

    # 只处理没有 categories 的 front matter
    fm_match = re.match(r'^---(.*?)---', content, re.DOTALL)
    if not fm_match:
        return

    front_matter = fm_match.group(1)
    if 'categories:' in front_matter:
        return

    # 在 front matter 末尾加上 categories
    new_fm = front_matter.rstrip() + '\ncategories: ["未分类"]\n'
    new_content = content.replace(front_matter, new_fm, 1)

    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    print(f'已为 {file_path} 添加 categories: ["未分类"]')

def main():
    for root, dirs, files in os.walk('content'):
        for file in files:
            if file.endswith('.md'):
                add_default_category_to_md(os.path.join(root, file))

if __name__ == '__main__':
    main() 