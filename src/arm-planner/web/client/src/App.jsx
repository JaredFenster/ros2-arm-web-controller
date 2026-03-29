import { useState, useEffect, useRef } from 'react';

function App() {
  return (
    <div>
      <h1>Arm Web Controller</h1>
      <CameraFeed />
    </div>
  );
}

function CameraFeed() {
  const [imageSrc, setImageSrc] = useState(null);
  const prevUrl = useRef(null);

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:3001');
    ws.binaryType = 'blob';

    ws.onmessage = (event) => {
      if (prevUrl.current) {
          URL.revokeObjectURL(prevUrl.current) // free memory from last frame
      }
      const url = URL.createObjectURL(event.data);
      prevUrl.current = url;
      setImageSrc(url);
    };

    return () => ws.close();
  }, []);

  return <img src={imageSrc} alt="camera feed" />
}


function Button() {
  const [count, setCount] = useState(0);

  function handleClick() {
    setCount(count + 1);
  }

  return (
    <button onClick={handleClick}>
      Clicked {count} times
    </button>
  );
}

export default App;
