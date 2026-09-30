"""Small correctness fixtures for the host converter; no performance workloads."""
import importlib.util
import tempfile
import unittest
from pathlib import Path
import numpy as np

spec = importlib.util.spec_from_file_location('robot_mesh', Path(__file__).with_name('robot-mesh.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
RobotMesh = module.RobotMesh


class RobotMeshTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.dae = self.root / 'part.dae'
        self.dae.write_text('''<COLLADA><asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset>
        <library_geometries><geometry id="g"><mesh>
          <source id="p"><float_array id="pa" count="9">0 0 0 1 0 0 0 1 0</float_array>
            <technique_common><accessor source="#pa" count="3" stride="3"/></technique_common></source>
          <source id="n"><float_array id="na" count="9">0 0 1 0 0 1 0 0 1</float_array>
            <technique_common><accessor source="#na" count="3" stride="3"/></technique_common></source>
          <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
          <triangles count="1"><input semantic="VERTEX" source="#v" offset="0"/>
            <input semantic="NORMAL" source="#n" offset="1"/><p>0 0 1 1 2 2</p></triangles>
        </mesh></geometry></library_geometries>
        <library_visual_scenes><visual_scene id="s"><node>
          <matrix>1 0 0 0.2 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
          <instance_geometry url="#g"/></node></visual_scene></library_visual_scenes>
        <scene><instance_visual_scene url="#s"/></scene></COLLADA>''')

    def test_fixed_joint_visual_origin_and_metric_mesh_are_composed_once(self):
        urdf = self.root / 'robot.urdf'
        urdf.write_text(f'''<robot><link name="base"/><link name="inertia"><visual>
          <origin xyz="0.5 0 0" rpy="0 0 {np.pi}"/>
          <geometry><mesh filename="{self.dae.as_uri()}"/></geometry></visual></link>
          <joint type="fixed"><parent link="base"/><child link="inertia"/>
          <origin xyz="1 0 0" rpy="0 0 {np.pi}"/></joint></robot>''')
        manifest = RobotMesh.export(urdf, {'base'}, self.root / 'assets', self.root, {'inertia'})
        self.assertEqual(manifest[0]['frame'], 'base')
        np.testing.assert_allclose(manifest[0]['bounds'], [[0.7, 0, 0], [1.7, 1, 0]], atol=1e-6)
        again = RobotMesh.export(urdf, {'base'}, self.root / 'assets', self.root, {'inertia'})
        self.assertEqual(manifest, again)
        urdf.write_text(urdf.read_text().replace('type="fixed"', 'type="revolute"'))
        with self.assertRaises(ValueError):
            RobotMesh.export(urdf, {'base'}, self.root / 'assets', self.root, {'inertia'})

    def test_material_binding_keeps_distinct_diffuse_parts(self):
        text = self.dae.read_text()
        a = text.index('<triangles'); b = text.index('</triangles>', a) + len('</triangles>')
        triangle = text[a:b]
        text = text[:a] + triangle.replace('count="1"', 'count="1" material="blue"') + triangle.replace('count="1"', 'count="1" material="gray"') + text[b:]
        text = text.replace('<library_geometries>', """<library_effects>
          <effect id="eb"><profile_COMMON><technique><phong><diffuse><color>0.1 0.4 0.8 1</color></diffuse></phong></technique></profile_COMMON></effect>
          <effect id="eg"><profile_COMMON><technique><phong><diffuse><color>0.6 0.6 0.6 1</color></diffuse></phong></technique></profile_COMMON></effect>
          </library_effects><library_materials><material id="mb"><instance_effect url="#eb"/></material>
          <material id="mg"><instance_effect url="#eg"/></material></library_materials><library_geometries>""")
        text = text.replace('<instance_geometry url="#g"/>', """<instance_geometry url="#g"><bind_material><technique_common>
          <instance_material symbol="blue" target="#mb"/><instance_material symbol="gray" target="#mg"/>
          </technique_common></bind_material></instance_geometry>""")
        self.dae.write_text(text)
        parts = RobotMesh.collada(self.dae, np.eye(4))
        self.assertEqual([p['rgba'] for p in parts], [[0.1,0.4,0.8,1], [0.6,0.6,0.6,1]])
        self.assertEqual(sum(len(p['mesh']['lods'][0]['indices']) for p in parts), 6)
        np.testing.assert_allclose(parts[0]['mesh']['positions'], parts[1]['mesh']['positions'])

    def test_reflection_winding_normals_and_unsupported_input(self):
        mesh = RobotMesh.collada(self.dae, np.diag([-2., 3., 1., 1.]))[0]['mesh']
        indices = mesh['lods'][0]['indices']
        positions = np.array(mesh['positions'])[indices]
        normal = np.cross(positions[1] - positions[0], positions[2] - positions[0])
        self.assertGreater(normal[2], 0)
        np.testing.assert_allclose(mesh['normals'], [[0, 0, 1]] * 3)
        self.dae.write_text(self.dae.read_text().replace('Z_UP', 'Y_UP'))
        with self.assertRaises(ValueError):
            RobotMesh.collada(self.dae, np.eye(4))
        self.dae.write_text('<!DOCTYPE x><x/>')
        with self.assertRaises(ValueError):
            RobotMesh.xml(self.dae)

    def test_triangular_polylist_matches_triangles_and_rejects_other_faces(self):
        expected = RobotMesh.collada(self.dae, np.eye(4))
        original = self.dae.read_text()
        polylist = original.replace('<triangles count="1">', '<polylist count="1"><vcount>3</vcount>').replace('</triangles>', '</polylist>')
        self.dae.write_text(polylist.replace('</mesh>', '<lines count="0"/></mesh>'))
        with self.assertWarnsRegex(UserWarning, 'line primitives omitted'):
            self.assertEqual(RobotMesh.collada(self.dae, np.eye(4)), expected)
        for counts in ('4', '3 3', ''):
            self.dae.write_text(polylist.replace('<vcount>3</vcount>', f'<vcount>{counts}</vcount>'))
            with self.assertRaisesRegex(ValueError, 'triangular polylist'):
                RobotMesh.collada(self.dae, np.eye(4))


if __name__ == '__main__':
    unittest.main()
