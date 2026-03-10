/**
 * 舵机控制面板 - 简单静态文件服务器
 * 所有串口通信通过浏览器 Web Serial API 完成
 */
const express = require('express');
const path = require('path');

const app = express();
app.use(express.static(path.join(__dirname, 'web')));
app.get('/', (req, res) => res.sendFile(path.join(__dirname, 'web', 'index.html')));

const PORT = 8000;
app.listen(PORT, () => {
    console.log('\n' + '='.repeat(50));
    console.log('  舵机控制面板 (Web Serial API)');
    console.log('='.repeat(50));
    console.log(`\n  打开浏览器访问: http://localhost:${PORT}`);
    console.log('  需要使用 Chrome 或 Edge 浏览器');
    console.log('  按 Ctrl+C 停止\n');
});
