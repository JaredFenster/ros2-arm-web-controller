const express = require('express');
const { WebSocketServer } = require('ws');

const app = express();
const PORT = 3001;

app.use(express.json());

const server = app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});

const wss = new WebSocketServer({ server });

let bridgeSocket = null;
const browserClients = new Set();

wss.on('connection', (ws, req) => {
  if (req.url === '/bridge') {
    // this is ros_bridge.py connecting
    console.log('ROS bridge connected');
    bridgeSocket = ws;

    ws.on('message', (data) => {
      // forward the jpeg bytes to all browser clients
      for (const client of browserClients) {
        if (client.readyState === 1) {
          client.send(data);
        }
      }
    });

    ws.on('close', () => {
      console.log('ROS bridge disconnected');
      bridgeSocket = null;
    });

  } else {
    // this is a browser connecting
    console.log('Browser client connected');
    browserClients.add(ws);

    ws.on('close', () => {
      browserClients.delete(ws);
    });
  }
});
