"""
Sirius ROS2 Launch Manager - Alias Parser
bash_alias2ファイルのパーサー
"""

import re


def parse_bash_aliases(alias_file_path):
    """bash_alias2ファイルからエイリアスとプリセットを解析（複数行対応）"""
    groups = {}
    presets = []
    current_group = "その他"
    current_description = ""
    current_preset_name = None
    
    try:
        with open(alias_file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        lines = content.split('\n')
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            
            if not line:
                current_description = ""
                i += 1
                continue
            
            # プリセット名の検出
            if line.startswith('# PRESET:'):
                current_preset_name = line.replace('# PRESET:', '').strip()
                i += 1
                continue
            
            # プリセットアイテムの検出
            if line.startswith('# PRESET_ITEMS:') and current_preset_name:
                items_str = line.replace('# PRESET_ITEMS:', '').strip()
                items = [item.strip() for item in items_str.split(',')]
                presets.append((current_preset_name, items))
                current_preset_name = None
                i += 1
                continue
            
            if line.startswith('# GROUP:'):
                current_group = line.replace('# GROUP:', '').strip()
                if current_group not in groups:
                    groups[current_group] = []
                current_description = ""
                i += 1
                continue
            
            if line.startswith('#') and not line.startswith('# GROUP:'):
                current_description = line.lstrip('#').strip()
                i += 1
                continue
            
            if line.startswith('alias '):
                # 複数行のエイリアスを結合
                full_line = line
                while full_line.endswith('\\') and i + 1 < len(lines):
                    i += 1
                    full_line = full_line[:-1] + ' ' + lines[i].strip()
                
                # エイリアスをパース
                match = re.match(r"alias\s+([^=]+)='(.+)'", full_line, re.DOTALL)
                if match:
                    alias_name = match.group(1).strip()
                    command = match.group(2).strip()
                    
                    if alias_name in ['install_packages']:
                        i += 1
                        continue
                    
                    # src && を実際のコマンドに展開
                    if command.startswith('src && '):
                        command = command.replace('src && ', 
                            'cd ~/sirius_jazzy_ws && source ~/sirius_jazzy_ws/install/setup.bash && ')
                    
                    description = current_description if current_description else alias_name
                    
                    if current_group not in groups:
                        groups[current_group] = []
                    groups[current_group].append((alias_name, command, description))
                    
                    current_description = ""
            
            i += 1
    
    except Exception as e:
        print(f"エイリアスファイルの読み込みエラー: {e}")
    
    return groups, presets
