package org.osm2world.world.modules;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.osm2world.map_data.data.MapWaySegment;
import org.osm2world.map_elevation.data.GroundState;
import org.osm2world.math.VectorXYZ;
import org.osm2world.math.shapes.TriangleXYZ;
import org.osm2world.scene.material.Material;
import org.osm2world.world.data.NoOutlineWaySegmentWorldObject;
import org.osm2world.world.data.ProceduralWorldObject;

/** Metric harbour triangles relative to the first node of their owning OSM way. */
final class HarbourPatch extends NoOutlineWaySegmentWorldObject implements ProceduralWorldObject {
	private static final int FORMAT_MAGIC = 0x41574a32;
	private static final Material[] MATERIALS = {
		HarbourModule.CAP, HarbourModule.WALL, HarbourModule.TIDE,
		HarbourModule.material(151, 151, 143), HarbourModule.material(118, 119, 113),
		HarbourModule.material(84, 88, 84), HarbourModule.material(98, 100, 94),
		HarbourModule.material(74, 79, 77), HarbourModule.BED, HarbourModule.WATER
	};

	record Batch(int material, float[] vertices) {}

	private final List<Batch> batches;

	HarbourPatch(MapWaySegment way, List<Batch> batches) {
		super(way);
		this.batches = batches;
	}

	@Override
	public GroundState getGroundState() {
		return GroundState.ON;
	}

	static Map<Long, List<Batch>> read(URI uri) {
		Map<Long, List<Batch>> result = new HashMap<>();
		if (uri == null) return result;
		try (var input = new DataInputStream(new BufferedInputStream(new FileInputStream(new File(uri))))) {
			if (input.readInt() != FORMAT_MAGIC) throw new IOException("Invalid harbour geometry header");
			int count = input.readInt();
			if (count < 0) throw new IOException("Negative harbour batch count");
			for (int i = 0; i < count; i++) {
				long id = input.readLong();
				int material = input.readUnsignedByte();
				int size = input.readInt();
				if (material >= MATERIALS.length || size < 0 || size > 300000) {
					throw new IOException("Invalid harbour batch");
				}
				float[] vertices = new float[Math.multiplyExact(size, 9)];
				for (int j = 0; j < vertices.length; j++) vertices[j] = input.readFloat();
				result.computeIfAbsent(id, k -> new ArrayList<>()).add(new Batch(material, vertices));
			}
			if (input.read() != -1) throw new IOException("Unexpected trailing harbour geometry");
		} catch (IOException ex) {
			throw new IllegalStateException("Cannot load metric harbour polygons", ex);
		}
		return result;
	}

	@Override
	public void buildMeshesAndModels(Target target) {
		var origin = getStartPosition().xyz(0);
		for (Batch batch : batches) {
			var coords = batch.vertices();
			List<TriangleXYZ> triangles = new ArrayList<>(coords.length / 9);
			for (int i = 0; i < coords.length; i += 9) {
				VectorXYZ a = origin.add(coords[i], coords[i + 1], coords[i + 2]);
				VectorXYZ b = origin.add(coords[i + 3], coords[i + 4], coords[i + 5]);
				VectorXYZ c = origin.add(coords[i + 6], coords[i + 7], coords[i + 8]);
				if (b.subtract(a).cross(c.subtract(a)).length() >= 2e-6) {
					triangles.add(new TriangleXYZ(a, b, c));
				}
			}
			if (!triangles.isEmpty()) {
				target.drawTriangles(MATERIALS[batch.material()], triangles, List.of());
			}
		}
	}
}
