import { describe, it, expect } from 'vitest'
import { VERSION } from '../src/index'
describe('tracker package', () => { it('exports a version', () => { expect(VERSION).toBe('0.0.1') }) })
