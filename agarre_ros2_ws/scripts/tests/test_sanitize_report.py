"""Regresiones de privacidad con valores ficticios, sin secretos reales."""

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'sanitize_report.py'
SPEC = importlib.util.spec_from_file_location('sanitize_report', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SanitizeReportTests(unittest.TestCase):
    def test_colcon_keeps_command_and_result(self):
        raw = (
            "Command: {'cmd': ['cmake', '--build', '.'], "
            "'env': OrderedDict({'VSCODE_CLI_REQUIRE_TOKEN': 'fake-value', "
            "'PATH': '/tmp/example', 'SPECIAL': 'quoted } ( value'}), 'shell': False}\n"
            "Finished <<< ur5_tools [1.0s]\n"
        )
        clean = MODULE.sanitize_text(raw)
        self.assertEqual(clean, (
            "Command: {'cmd': ['cmake', '--build', '.'], "
            f"'env': {MODULE.ENV_MARKER}, 'shell': False}}\n"
            "Finished <<< ur5_tools [1.0s]\n"
        ))

    def test_nested_multiline_environment(self):
        raw = "env={'A': ['fake', {'B': 'value'}],\n'C': 'value'}\nresult=PASS\n"
        self.assertEqual(MODULE.sanitize_text(raw),
                         f'env={MODULE.ENV_MARKER}\nresult=PASS\n')

    def test_patch_prefix_and_line_count_are_preserved(self):
        raw = "-Command: {'env': {'SECRET': 'fake'}, 'shell': False}\n+result=PASS\n"
        clean = MODULE.sanitize_text(raw)
        self.assertTrue(clean.startswith("-Command: {'env': "))
        self.assertEqual(raw.count('\n'), clean.count('\n'))
        self.assertTrue(clean.endswith('+result=PASS\n'))

    def test_session_values_and_quoted_spaces(self):
        raw = "export API_KEY=fake\nSSH_CONNECTION='example 123 example 456'\n"
        clean = MODULE.sanitize_text(raw)
        self.assertNotIn('fake', clean)
        self.assertNotIn('example', clean)
        self.assertEqual(clean, MODULE.sanitize_text(clean))

    def test_truncated_environment_does_not_leak(self):
        raw = "diagnostic env={'PATH': 'fake', 'NEXT': 'truncated"
        self.assertEqual(MODULE.sanitize_text(raw), f'diagnostic env={MODULE.ENV_MARKER}')

    def test_attributions_and_robot_config_are_unchanged(self):
        raw = (
            'author: "Claude Code"\nCo-Authored-By: Claude <noreply@anthropic.com>\n'
            'ChatGPT: asistencia declarada en el anexo B.\n'
            'ROS_DOMAIN_ID=42\nPANEL_QT_PLATFORM=xcb\n'
            'os.environ.get("SSH_CONNECTION")\n'
        )
        self.assertEqual(MODULE.sanitize_text(raw), raw)

    def test_cli_check_does_not_print_values_or_modify_files(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'report.md'
            path.write_text('API_KEY=fictitious-secret\n', encoding='utf-8')
            run = subprocess.run([sys.executable, str(SCRIPT), '--check', str(path)],
                                 capture_output=True, text=True)
            self.assertEqual(run.returncode, 1)
            self.assertNotIn('fictitious-secret', run.stdout + run.stderr)
            self.assertEqual(path.read_text(), 'API_KEY=fictitious-secret\n')
            subprocess.run([sys.executable, str(SCRIPT), '--in-place', str(path)],
                           check=True, capture_output=True)
            run = subprocess.run([sys.executable, str(SCRIPT), '--check', str(path)],
                                 capture_output=True, text=True)
            self.assertEqual(run.returncode, 0)

    def test_pdf_is_never_rewritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'reference.pdf'
            data = b'%PDF-1.7\nAPI_KEY=fake\n'
            path.write_bytes(data)
            subprocess.run([sys.executable, str(SCRIPT), '--in-place', str(path)],
                           check=True, capture_output=True)
            self.assertEqual(path.read_bytes(), data)

    def test_tracked_report_scope_keeps_archived_source_code(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            names = ['report/diagnostic.md', 'reports/example.py',
                     'build/colcon_command_prefix_build.sh.env', 'src/module.py']
            for name in names:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('example\n')
            subprocess.run(['git', '-C', str(root), 'add', '.'], check=True)
            self.assertEqual(set(MODULE.tracked_reports(root)),
                             {root / names[0], root / names[2]})

    def test_generator_publishes_only_sanitized_markdown(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory) / 'workspace'
            scripts = workspace / 'scripts'
            scripts.mkdir(parents=True)
            generator = scripts / 'generar_base_conocimiento_tfm.sh'
            shutil.copy2(SCRIPT.parent / generator.name, generator)
            shutil.copy2(SCRIPT, scripts / SCRIPT.name)
            logs = workspace / 'log'
            logs.mkdir()
            (logs / 'ros2_launch.log').write_text(
                "Command: {'env': OrderedDict({'API_KEY': 'fake-sensitive-value'}), "
                "'shell': False}\nFinished <<< example\n"
            )
            for option in ('--out-dir', '--out-dir='):
                with self.subTest(option=option):
                    output = Path(directory) / ('public output ' + option)
                    args = [str(output)] if option == '--out-dir' else []
                    option_arg = option if args else option + str(output)
                    run = subprocess.run(
                        ['bash', str(generator), '--no-pdf', option_arg, *args],
                        capture_output=True, text=True, timeout=30,
                        env={**os.environ, 'REPORTS_BASE': str(workspace / 'unused')},
                    )
                    self.assertEqual(run.returncode, 0, run.stderr)
                    self.assertNotIn('fake-sensitive-value', run.stdout + run.stderr)
                    generated = list(output.glob('*.md'))
                    self.assertEqual(len(generated), 1)
                    content = generated[0].read_text()
                    self.assertNotIn('fake-sensitive-value', content)
                    self.assertIn(MODULE.ENV_MARKER, content)
                    self.assertIn('Finished <<< example', content)
                    self.assertIn('Claude Code', content)
                    self.assertFalse((workspace / 'unused').exists())
                    for temporary in output.glob('.tmp_base_*'):
                        self.assertEqual(temporary.stat().st_mode & 0o777, 0o700)
                        self.assertFalse((temporary / 'documento_sin_sanear.md').exists())


if __name__ == '__main__':
    unittest.main()
