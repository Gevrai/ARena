import js from '@eslint/js'
import tseslint from 'typescript-eslint'

export default tseslint.config(
  { ignores: ['dist', 'node_modules', '.superpowers'] },
  js.configs.recommended,
  ...tseslint.configs.strict,
  {
    files: ['packages/tracker/**/*.ts'],
    rules: {
      'no-restricted-imports': [
        'error',
        {
          patterns: [
            { group: ['three', 'three/*'], message: 'packages/tracker must not depend on three.' },
            { group: ['../../../*', 'src/*'], message: 'packages/tracker must not import from outside itself.' },
          ],
        },
      ],
    },
  },
)
