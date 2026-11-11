const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');

let portSelectCallback = null;

function createWindow () {
  const win = new BrowserWindow({
    width: 1280,
    height: 800,
    icon: path.join(__dirname, 'assets', 'icon.png'),
    title: 'Robot Arm Controller',
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false,
      webSecurity: false
    }
  });

  win.setMenuBarVisibility(false);

  // 🟢 ดักจับคำสั่งเชื่อมต่อ USB เพื่อดึงรายชื่อพอร์ตมาส่งให้ Dropdown (แทนการใช้ Popup)
  win.webContents.session.on('select-serial-port', (event, portList, webContents, callback) => {
    event.preventDefault(); // ปิดการแจ้งเตือน Popup ดั้งเดิม
    win.webContents.send('update-port-list', portList); // ส่งรายชื่อพอร์ตไปที่ index.html
    portSelectCallback = callback; // พักการเชื่อมต่อไว้ รอจนกว่าผู้ใช้จะกดปุ่ม Connect
  });

  // 🟢 อนุญาตสิทธิ์การเข้าถึงฮาร์ดแวร์ Serial Port ใน Electron
  win.webContents.session.setPermissionCheckHandler((webContents, permission) => {
    if (permission === 'serial') return true;
    return false;
  });
  win.webContents.session.setDevicePermissionHandler((details) => {
    if (details.deviceType === 'serial') return true;
    return false;
  });

  win.loadFile('index.html');
}

app.commandLine.appendSwitch('enable-experimental-web-platform-features');

app.whenReady().then(() => {
  createWindow();
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

// 🟢 รับค่า ID ของพอร์ตที่ผู้ใช้เลือกจากหน้าเว็บ แล้วสั่งเริ่มการเชื่อมต่อทันที
ipcMain.on('connect-to-port', (event, portId) => {
  if (portSelectCallback) {
    portSelectCallback(portId);
    portSelectCallback = null;
  }
});

// 🟢 รีเซ็ตคำสั่งหากผู้ใช้กดปุ่ม Scan ซ้ำ
ipcMain.on('cancel-port-request', () => {
  if (portSelectCallback) {
    portSelectCallback('');
    portSelectCallback = null;
  }
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});