#!/usr/bin/env python3
import sys

def parse_scene(filepath):
    print(f"Parsing {filepath}...")
    with open(filepath, 'r') as f:
        content = f.read()
        
    # Split content by Unity YAML document separator
    docs = content.split('--- !u!')
    
    gameobjects = {}
    components = {}
    
    for doc in docs:
        if not doc.strip():
            continue
        lines = doc.split('\n')
        header = lines[0].strip()
        doc_id = header.split('&')[-1]
        
        # Parse GameObject
        if header.startswith('1 '):
            name = None
            for line in lines:
                if 'm_Name:' in line:
                    name = line.split('m_Name:')[-1].strip()
            gameobjects[doc_id] = {'name': name, 'components': []}
            
        # Parse components or MonoBehaviour
        elif header.startswith('114 '): # MonoBehaviour
            go_id = None
            script_guid = None
            for line in lines:
                if 'm_GameObject:' in line:
                    go_id = line.split('fileID:')[-1].replace('}', '').strip()
                if 'guid:' in line:
                    script_guid = line.split('guid:')[-1].split(',')[0].strip()
            if go_id:
                components.setdefault(go_id, []).append(('MonoBehaviour', script_guid))
                
        # PrefabInstance
        elif header.startswith('1001 '):
            prefab_id = doc_id
            # find modified names
            name = None
            for line in lines:
                if 'propertyPath: m_Name' in line:
                    idx = lines.index(line)
                    val_line = lines[idx+1]
                    if 'value:' in val_line:
                        name = val_line.split('value:')[-1].strip()
            if name:
                gameobjects[prefab_id] = {'name': name, 'components': []}

    print("\nGameObjects found in scene file:")
    for go_id, info in gameobjects.items():
        if 'NPC' in str(info['name']):
            print(f"ID: {go_id}, Name: {info['name']}")
            
parse_scene('/home/kotantu-desktop/sirius-unity-sim/Assets/Scenes/natural_world1.unity')
