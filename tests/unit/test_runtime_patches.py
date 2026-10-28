"""Overlapping runtime patches rebuild idempotently without overwriting edits."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import subprocess
import tempfile
import unittest
from mujoco_ros2_manipulation.build import apply_patch_series


class RuntimePatchTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.source.mkdir()
        self.file = self.source / "file.txt"
        self.file.write_text("base\n")
        for command in [["git", "init", "--quiet"], ["git", "add", "file.txt"],
                        ["git", "-c", "user.name=Test", "-c", "user.email=test@example.invalid",
                         "commit", "--quiet", "-m", "fixture"]]:
            subprocess.run(command, cwd=self.source, check=True, stdout=subprocess.DEVNULL)
        self.patches = []
        for index, (before, after) in enumerate([("base", "first"), ("first", "final")]):
            path = self.root / f"{index}.patch"
            path.write_text(f"--- a/file.txt\n+++ b/file.txt\n@@ -1 +1 @@\n-{before}\n+{after}\n")
            self.patches.append(path)

    def test_fresh_partial_and_repeated_application(self):
        apply_patch_series(self.source, self.patches)
        self.assertEqual(self.file.read_text(), "final\n")
        apply_patch_series(self.source, self.patches)
        self.assertEqual(self.file.read_text(), "final\n")
        self.file.write_text("first\n")
        apply_patch_series(self.source, self.patches)
        self.assertEqual(self.file.read_text(), "final\n")

    def test_user_change_is_preserved_before_any_real_patch(self):
        self.file.write_text("user edit\n")
        unrelated = self.source / "unrelated.txt"
        unrelated.write_text("keep")
        with self.assertRaisesRegex(ValueError, "left unchanged"):
            apply_patch_series(self.source, self.patches)
        self.assertEqual(self.file.read_text(), "user edit\n")
        self.assertEqual(unrelated.read_text(), "keep")

    def test_new_fifth_patch_accepts_all_old_prefixes_without_rewriting_user_edits(self):
        patches=list(self.patches)
        for index,(before,after) in enumerate((("final","third"),("third","fourth")),2):
            path=self.root/f"{index}.patch"
            path.write_text(f"--- a/file.txt\n+++ b/file.txt\n@@ -1 +1 @@\n-{before}\n+{after}\n")
            patches.append(path)
        final=self.root/'4.patch'
        final.write_text('--- /dev/null\n+++ b/node.cpp\n@@ -0,0 +1 @@\n+ordered shutdown\n')
        patches.append(final)
        node=self.source/'node.cpp'
        def apply(selected):
            with redirect_stdout(io.StringIO()):
                apply_patch_series(self.source,selected)
        for count in range(6):
            with self.subTest(already_applied=count):
                self.file.write_text('base\n');node.unlink(missing_ok=True)
                apply(patches[:count])
                apply(patches)
                self.assertEqual(self.file.read_text(),'fourth\n')
                self.assertEqual(node.read_text(),'ordered shutdown\n')
                before=(self.file.read_bytes(),node.read_bytes())
                apply(patches)
                self.assertEqual((self.file.read_bytes(),node.read_bytes()),before)
        node.write_text('user node edit\n')
        before=(self.file.read_bytes(),node.read_bytes())
        with self.assertRaisesRegex(ValueError,'left unchanged'):
            apply(patches)
        self.assertEqual((self.file.read_bytes(),node.read_bytes()),before)
