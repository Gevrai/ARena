import { defineConfig } from 'vitest/config'
import basicSsl from '@vitejs/plugin-basic-ssl'

export default defineConfig({
  base: '/ARena/',
  plugins: [basicSsl()],
  worker: { format: 'es' },
  build: {
    rollupOptions: {
      input: { main: 'index.html', marker: 'marker/index.html', demo: 'demo/tracker.html' },
    },
  },
  test: {
    include: ['packages/**/test/**/*.test.ts', 'src/**/*.test.ts'],
  },
})
