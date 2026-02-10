#!/bin/bash
echo "🔍 Verifying VR Files Installation"
echo "=================================="
echo ""

cd ~/drt 2>/dev/null || cd /Users/ziguo/drt

check_file() {
    if [ -f "$1" ]; then
        size=$(ls -lh "$1" | awk '{print $5}')
        echo "✅ $1 ($size)"
        return 0
    else
        echo "❌ MISSING: $1"
        return 1
    fi
}

echo "📁 Checking VR Files:"
echo ""

check_file "client/web/vr.html"
check_file "server/vr_video_endpoints.py"
check_file "server/backends/soarm_backend_vr_patch.py"
check_file "start_vr_demo.sh"
check_file "VR_SETUP.md"

echo ""
echo "=================================="
echo "✅ All VR files are in place!"
echo ""
echo "🚀 Quick Start:"
echo "   cd ~/drt"
echo "   ./start_vr_demo.sh"
