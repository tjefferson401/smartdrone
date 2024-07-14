// import { defineConfig } from 'vite'
// import react from '@vitejs/plugin-react'

// // https://vitejs.dev/config/
// export default defineConfig({
//   plugins: [react()],
// })


import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import fs from 'fs';

// Path to the certificate and key files
const httpsKeyPath = '/your/path/to/cert-key.pem';
const httpsCertPath = '/your/path/to/cert.pem';

export default defineConfig({
  plugins: [react()],
  server: {
    https: {
      key: fs.readFileSync(httpsKeyPath),
      cert: fs.readFileSync(httpsCertPath)
    },
    host: true // Listens on all local IPs
  }
});