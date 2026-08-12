import os
import sys
import importlib.util

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_ROOT = os.path.join(SCRIPT_DIR, 'src', 'sdk_server')

sys.path.insert(0, SDK_ROOT)
os.chdir(SDK_ROOT)

spec = importlib.util.spec_from_file_location('sdk_main', os.path.join(SDK_ROOT, 'main.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
import uvicorn
uvicorn.run(mod.app, host='0.0.0.0', port=8000)
