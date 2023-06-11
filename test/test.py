import os

os.system("cd ~/SE/team03-project; git pull origin main > tmp")
with open('/home/jinghongbin/SE/team03-project/tmp', encoding='utf-8') as f:
    if f.read().startswith('已经是最新的。'):
        ...
    else: