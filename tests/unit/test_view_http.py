"""View endpoints remain bound to one selected scene and same-origin POST token."""
import base64
from email.message import Message
import hashlib
from importlib.resources import files
import io
import json
import re
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
from mujoco_ros2_manipulation.operator_console.http_server import Handler, IMPORT_MAP_CSP_HASH


BINDING='a'*64
ASSET='b'*64


class Host:
    def manifest_bytes(self,identifier):
        if identifier!=BINDING:raise RuntimeError('Old binding')
        return b'{"manifest":true}'
    def pose_envelope(self,identifier):
        if identifier!=BINDING:raise RuntimeError('Old binding')
        return None
    def asset_bytes(self,digest):
        return b'geometry' if digest==ASSET else None


def handler(path,*,method='GET',payload=None,token='secret',origin='http://127.0.0.1:8890'):
    value=object.__new__(Handler);value.path=path;value.headers=Message()
    value.headers['Host']='127.0.0.1:8890'
    value.headers['Origin']=origin;value.headers['X-Operator-Token']=token
    app=NS(views=NS(host=Host(),select=Mock(return_value={'mode':'model_preview'}),seek=Mock(return_value={'preview_time_sec':.2})))
    value.server=NS(application=app,server_address=('127.0.0.1',8890),token='secret')
    response=Mock();value.respond=response
    if method=='POST':
        raw=json.dumps(payload).encode();value.headers['Content-Type']='application/json';value.headers['Content-Length']=str(len(raw));value.rfile=io.BytesIO(raw)
    return value,response,app


class ViewHTTPTests(unittest.TestCase):
    def test_manifest_and_waiting_pose(self):
        value,respond,_=handler('/api/view/manifest?binding_id='+BINDING);value.do_GET()
        self.assertEqual(respond.call_args.args[0],b'{"manifest":true}')
        self.assertEqual(respond.call_args.kwargs['mime'],'application/json; charset=utf-8')
        value,respond,_=handler('/api/view/pose?binding_id='+BINDING);value.do_GET();self.assertEqual(respond.call_args.args,(b'',204))

    def test_wrong_or_ambiguous_binding_is_rejected(self):
        for query,status in [('binding_id='+'c'*64,409),('binding_id='+BINDING+'&binding_id='+BINDING,400),('',400),('binding_id=../x',400),('binding_id='+BINDING+'&x=1',400)]:
            value,respond,_=handler('/api/view/manifest?'+query);value.do_GET();self.assertEqual(respond.call_args.args[1],status)

    def test_assets_use_only_current_hash_catalog(self):
        value,respond,_=handler('/api/view/assets/'+ASSET);value.do_GET();self.assertEqual(respond.call_args.args[0],b'geometry')
        for suffix,status in [('c'*64,404),('../source.json',400),('a'*63,400),('https:example',400)]:
            value,respond,_=handler('/api/view/assets/'+suffix);value.do_GET();self.assertEqual(respond.call_args.args[1],status)

    def test_disabled_host_has_no_pose_or_manifest(self):
        for endpoint in ('manifest','pose'):
            value,respond,app=handler('/api/view/'+endpoint+'?binding_id='+BINDING);app.views.host=None;value.do_GET();self.assertEqual(respond.call_args.args,(b'',204))

    def test_seek_passes_current_binding_to_owner(self):
        value,respond,app=handler('/api/view/seek',method='POST',payload={'time_sec':.2,'binding_id':BINDING});value.do_POST()
        app.views.seek.assert_called_once_with(.2,binding_id=BINDING);self.assertEqual(respond.call_args.args[1],202)
        value,respond,app=handler('/api/view/seek',method='POST',payload={'time_sec':.2});value.do_POST();self.assertEqual(respond.call_args.args[1],404);app.views.seek.assert_not_called()

    def test_view_mutations_require_origin_and_token(self):
        for token,origin in [('wrong','http://127.0.0.1:8890'),('secret','https://example.test')]:
            value,respond,app=handler('/api/view/select',method='POST',payload={'mode':'model_preview','source_id':'profile:fr5'},token=token,origin=origin);value.do_POST();self.assertEqual(respond.call_args.args[1],403);app.views.select.assert_not_called()

    def test_import_map_has_one_exact_csp_hash(self):
        page=files('mujoco_ros2_manipulation.operator_console').joinpath('static','index.html').read_text()
        maps=re.findall(r'<script type="importmap">(.*?)</script>',page,re.S);self.assertEqual(len(maps),1)
        digest=base64.b64encode(hashlib.sha256(maps[0].encode()).digest()).decode();self.assertEqual(digest,IMPORT_MAP_CSP_HASH)
        value=object.__new__(Handler);value.path='/';value.send_response=Mock();value.send_header=Mock();value.end_headers=Mock();value.wfile=io.BytesIO();value.respond(b'ok',mime='text/plain')
        policy=dict(value.send_header.call_args_list[index].args for index in range(value.send_header.call_count))['Content-Security-Policy']
        self.assertIn("'sha256-"+digest+"'",policy);self.assertNotIn('unsafe-inline',policy);self.assertNotIn('unsafe-eval',policy)

    def test_vendored_files_match_pinned_upstream_digests(self):
        root=files('mujoco_ros2_manipulation.operator_console').joinpath('static','vendor')
        manifest=json.loads(root.joinpath('vendor-manifest.json').read_text());self.assertEqual(manifest['name'],'three');self.assertEqual(manifest['version'],'0.186.1');self.assertEqual(manifest['license'],'MIT')
        for name,digest in manifest['files'].items():self.assertEqual(hashlib.sha256(root.joinpath(name).read_bytes()).hexdigest(),digest)


if __name__=='__main__':unittest.main()
