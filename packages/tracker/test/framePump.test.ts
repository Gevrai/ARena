import { describe, expect, it } from 'vitest'
import { FramePump } from '../src/worker/framePump'

describe('FramePump', () => {
  it('allows one frame in flight by default and counts drops', () => {
    const p = new FramePump({})
    expect(p.canSend()).toBe(true)
    p.markSent(1)
    expect(p.canSend()).toBe(false)
    // frames skipped while busy
    p.noteDropped()
    p.noteDropped()
    p.markDone(1, 100)
    expect(p.canSend()).toBe(true)
    expect(p.stats()).toMatchObject({ sent: 1, done: 1, dropped: 2 })
  })

  it('respects maxInFlight > 1', () => {
    const p = new FramePump({ maxInFlight: 2 })
    p.markSent(1)
    expect(p.canSend()).toBe(true)
    p.markSent(2)
    expect(p.canSend()).toBe(false)
    p.markDone(1, 10)
    expect(p.canSend()).toBe(true)
  })

  it('ignores unknown or duplicate markDone', () => {
    const p = new FramePump({})
    p.markDone(5, 0)
    p.markSent(1)
    p.markDone(1, 0)
    p.markDone(1, 0)
    expect(p.stats().done).toBe(1)
    expect(p.canSend()).toBe(true)
  })

  it('computes detectHz from completion timestamps over a sliding window', () => {
    const p = new FramePump({})
    expect(p.stats().detectHz).toBe(0)
    for (let i = 0; i < 11; i++) {
      p.markSent(i)
      p.markDone(i, i * 100) // 10 Hz
    }
    expect(p.stats().detectHz).toBeCloseTo(10, 5)
  })
})
