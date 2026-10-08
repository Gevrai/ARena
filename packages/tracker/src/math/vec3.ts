export type Vec3 = [number, number, number]

export const vec3Add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
export const vec3Sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
export const vec3Scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s]
export const vec3Dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
export const vec3Cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
]
export const vec3Length = (a: Vec3): number => Math.hypot(a[0], a[1], a[2])
export function vec3Normalize(a: Vec3): Vec3 {
  const l = vec3Length(a)
  return l === 0 ? [0, 0, 0] : [a[0] / l, a[1] / l, a[2] / l]
}
export const vec3Lerp = (a: Vec3, b: Vec3, t: number): Vec3 => [
  a[0] + (b[0] - a[0]) * t,
  a[1] + (b[1] - a[1]) * t,
  a[2] + (b[2] - a[2]) * t,
]
