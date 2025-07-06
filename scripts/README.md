# Hugo Front Matter 自动填充工具

这个工具可以自动为你的 Hugo 文章添加完整的 Front Matter 信息，包括：

- 📅 **创建时间** (`date`) - 从文件系统或 Git 历史获取
- 🔄 **修改时间** (`lastmod`) - 从文件系统或 Git 历史获取，支持使用当前时间
- 👤 **作者** (`author`) - 统一设置作者名称
- 📂 **分类** (`categories`) - 根据文件夹结构自动分类
- 📝 **标题** (`title`) - 从文件名自动生成

## 🆕 新增功能

### 时间更新功能
- `--update-dates` - 仅更新日期相关字段（date 和 lastmod）
- `--update-lastmod` - 仅更新最后修改时间
- `--use-current-time` - 使用当前时间作为修改时间

## 功能特点

### 1. 智能时间获取
- 优先使用 Git 提交历史的时间
- 如果 Git 不可用，则使用文件系统时间
- 支持 macOS 和 Linux 系统

### 2. 自动分类系统
根据你的项目结构，自动生成分类：

```
panda-blog/          → 技术文档
├── vpn/             → VPN技术  
├── ACM/             → 算法竞赛
├── macos/           → macOS
├── server/          → 服务器
└── 工具/            → 工具使用

想法/                → 创意想法
```

### 3. 智能标题生成
- 中文文件名：直接使用
- 英文文件名：转换为标题格式（如 `my-post.md` → `My Post`）

## 使用方法

### 基本用法

```bash
# 处理所有文章（默认目录：panda-blog 和 panda_wiki）
python3 scripts/enhanced_front_matter.py

# 预览模式（不实际修改文件）
python3 scripts/enhanced_front_matter.py --dry-run

# 强制更新所有文件（包括已有 Front Matter 的文件）
python3 scripts/enhanced_front_matter.py --force
```

### 高级用法

```bash
# 指定作者名称
python3 scripts/enhanced_front_matter.py --author "你的名字"

# 处理单个文件
python3 scripts/enhanced_front_matter.py --file "panda-blog/vpn/trojan-vpn.md"

# 指定内容目录
python3 scripts/enhanced_front_matter.py --content-dirs "content/posts/panda-blog" "content/posts/panda_wiki"
```

## 🤖 GitHub Actions 集成

### 自动化工作流

脚本已集成到 GitHub Actions 工作流中，这意味着：

1. **自动执行**：每次推送代码或定时任务时自动运行
2. **无需手动操作**：专注于文档内容，无需担心 Front Matter 格式
3. **处理多源内容**：自动处理本地内容和从 `panda_wiki` 仓库克隆的内容

### 工作流步骤

在 `.github/workflows/gh-pages.yml` 中，脚本会在以下步骤后执行：

1. ✅ 克隆 `panda_wiki` 仓库内容
2. 🔧 **自动生成 Front Matter**（新增）
3. 🏗️ 构建 Hugo 站点
4. 🚀 部署到 GitHub Pages

### 优势

- **专注内容创作**：只需写 Markdown 内容，无需手动添加 Front Matter
- **自动时间管理**：基于 Git 历史自动设置准确的创建时间
- **智能分类**：根据文件路径自动生成合适的分类
- **统一格式**：确保所有文章都有一致的 Front Matter 格式

### 本地开发

如果你想在本地测试，仍然可以使用：

```bash
# 本地运行脚本
python3 scripts/enhanced_front_matter.py --force

# 启动 Hugo 服务器
make serve
```

这样你就可以专注于文档内容创作，而不用担心 Front Matter 的格式问题！

## 生成的 Front Matter 示例

```yaml
---
title: "Trojan VPN 配置指南"
date: 2024-01-15T10:30:00+08:00
lastmod: 2024-01-20T14:45:00+08:00
author: "panda"
categories: ["技术文档", "VPN技术"]
---
```

## 配置说明

你可以在脚本中修改 `CONFIG` 字典来自定义：

```python
CONFIG = {
    "author": "panda",  # 默认作者
    "content_dirs": [   # 内容目录
        "panda-blog",
        "想法"
    ],
    "category_mapping": {  # 分类映射
        "panda-blog": "技术文档",
        "想法": "创意想法",
        "vpn": "VPN技术",
        # ... 更多映射
    }
}
```

## 注意事项

1. **备份重要文件**：首次使用前建议备份你的文章
2. **Git 仓库**：工具会尝试从 Git 历史获取时间，确保在 Git 仓库中运行
3. **编码支持**：支持 UTF-8 编码的中文文件名和内容
4. **现有 Front Matter**：默认会保留已有的 Front Matter，使用 `--force` 强制更新

## 与原有脚本的区别

这个增强版脚本相比原来的 `update_post_dates.py`：

- ✅ 添加了作者信息
- ✅ 添加了自动分类
- ✅ 添加了标题生成
- ✅ 更智能的更新逻辑（只更新缺失的字段）
- ✅ 更好的预览功能
- ✅ 更详细的输出信息

## 故障排除

### 常见问题

1. **权限错误**：确保对文件有读写权限
2. **编码错误**：确保文件是 UTF-8 编码
3. **Git 错误**：如果不在 Git 仓库中，会回退到文件系统时间
4. **路径错误**：确保指定的文件或目录存在

### 调试模式

使用 `--dry-run` 参数可以预览将要进行的更改，不会实际修改文件。 