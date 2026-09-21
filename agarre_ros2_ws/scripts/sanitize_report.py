#!/usr/bin/env python3
"""Omitir entornos de procesos y valores sensibles de informes publicables.

No modifica atribuciones, resultados experimentales ni los PDF de la memoria.
No es un escáner general de secretos: reconoce volcados de entorno de colcon
y asignaciones habituales de credenciales/datos de sesión en texto.
"""

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


ENV_MARKER = "'[ENTORNO OMITIDO POR PRIVACIDAD]'"
VALUE_MARKER = "'[DATO OMITIDO POR PRIVACIDAD]'"
REPORT_SUFFIXES = {
    '.md', '.txt', '.log', '.patch', '.diff', '.env', '.json', '.yaml', '.yml',
    '.html', '.csv', '.out', '.err',
}
ENV_START = re.compile(
    r"(?:['\"](?:env|environ|environment)['\"]\s*:\s*"
    r"|\b(?:env|environ|environment)\s*=\s*)"
    r"(?:(?:OrderedDict|environ|dict)\s*)?(?=[({\[])"
)
SESSION_VALUE = re.compile(
    r"(?P<prefix>(?<![\w])['\"]?"
    r"(?:[A-Z][A-Z0-9_]*(?:TOKEN|PASSWORD|PASSWD|SECRET|API_KEY)"
    r"|TOKEN|PASSWORD|PASSWD|SECRET|API_KEY|SSH_CONNECTION|SSH_CLIENT|SSH_AUTH_SOCK)"
    r"['\"]?\s*[:=]\s*)"
    r"(?:'(?:\\.|[^'\\\r\n])*'|\"(?:\\.|[^\"\\\r\n])*\"|[^\s,;}\]\r\n]+)"
)


def container_end(text, start):
    """Encontrar el cierre de un repr sin evaluar el contenido del registro."""
    closing = {'(': ')', '[': ']', '{': '}'}
    stack = []
    quote = None
    escaped = False
    for index in range(start, len(text)):
        char = text[index]
        if quote:
            if escaped:
                escaped = False
            elif char == '\\':
                escaped = True
            elif char == quote:
                quote = None
        elif char in "\"'":
            quote = char
        elif char in closing:
            stack.append(closing[char])
        elif char in ')]}':
            if not stack or char != stack.pop():
                break
            if not stack:
                return index + 1
    # Un volcado truncado no debe dejar valores sin filtrar a continuación.
    return len(text)


def sanitize_text(text):
    """Conservar el diagnóstico y sustituir únicamente datos de entorno/sesión."""
    chunks = []
    offset = 0
    while match := ENV_START.search(text, offset):
        prefix = text[match.start():match.end()]
        value_start = match.start() + re.search(r'[:=]\s*', prefix).end()
        end = container_end(text, match.end())
        chunks.extend((text[offset:value_start], ENV_MARKER))
        offset = end
    chunks.append(text[offset:])
    return SESSION_VALUE.sub(lambda m: m['prefix'] + VALUE_MARKER, ''.join(chunks))


def tracked_reports(root):
    """Enumerar informes versionados, incluidos parches y entornos de build."""
    result = subprocess.run(
        ['git', '-C', str(root), 'ls-files', '-z'],
        check=True, stdout=subprocess.PIPE,
    )
    for raw in result.stdout.split(b'\0'):
        if not raw:
            continue
        relative = Path(os.fsdecode(raw))
        if ((relative.parts[0] in {'report', 'reports', 'auditoria'}
             and relative.suffix.lower() in REPORT_SUFFIXES)
                or (relative.name.startswith('colcon_command_prefix_')
                    and relative.suffix == '.env')):
            yield root / relative


def rewrite(path, data):
    """Reemplazar el archivo atómicamente sin crear copias con secretos."""
    mode = path.stat().st_mode & 0o777
    descriptor, temporary = tempfile.mkstemp(prefix='.sanitize-', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(data)
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='fallar si hay datos que omitir')
    mode.add_argument('--in-place', action='store_true', help='sanear los archivos indicados')
    parser.add_argument('--tracked-reports', action='store_true', help='informes de Git')
    parser.add_argument('files', type=Path, nargs='*')
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[2]
    paths = list(args.files)
    if args.tracked_reports:
        paths.extend(tracked_reports(root))
    if not paths:
        if args.check or args.in_place or args.tracked_reports:
            parser.error('indica archivos o --tracked-reports con archivos disponibles')
        sys.stdout.write(sanitize_text(sys.stdin.read()))
        return 0
    if not (args.check or args.in_place):
        parser.error('usa --check o --in-place con archivos')
    changed = 0
    checked = 0
    for path in dict.fromkeys(paths):
        if not path.exists():
            continue
        if path.is_symlink() or not path.is_file():
            continue
        with path.open('rb') as stream:
            prefix = stream.read(8192)
            if b'\0' in prefix or prefix.startswith(b'%PDF'):
                continue
            data = prefix + stream.read()
        text = data.decode('utf-8', errors='surrogateescape')
        clean = sanitize_text(text)
        checked += 1
        if clean != text:
            changed += 1
            print(f'{"SANEAR" if args.check else "SANEADO"}: {path}', file=sys.stderr)
            if args.in_place:
                rewrite(path, clean.encode('utf-8', errors='surrogateescape'))
    print(f'Archivos de texto revisados: {checked}; con cambios: {changed}', file=sys.stderr)
    return int(args.check and changed > 0)


if __name__ == '__main__':
    sys.exit(main())
