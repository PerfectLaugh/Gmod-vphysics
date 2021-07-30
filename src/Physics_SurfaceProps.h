#ifndef PHYSICS_SURFACEPROPS_H
#define PHYSICS_SURFACEPROPS_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

enum {
	MATERIAL_INDEX_SHADOW = 0xF000,
};

class CSurface {
	public:
		unsigned short  m_name;
		surfacedata_t	data;
};

class CPhysicsSurfaceProps : public IPhysicsSurfaceProps {
	public:
								CPhysicsSurfaceProps();
								~CPhysicsSurfaceProps();
								
		int						ParseSurfaceData(const char *pFilename, const char *pTextfile);
		int						SurfacePropCount() const;

		int						GetSurfaceIndex(const char *pSurfacePropName) const;
		void					GetPhysicsProperties(int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity) const;

		surfacedata_t *			GetSurfaceData(int surfaceDataIndex);
		const char *			GetString(unsigned short stringTableIndex) const;

		const char *			GetPropName(int surfaceDataIndex) const;

		void					SetWorldMaterialIndexTable(int *pMapArray, int mapSize);

		void					GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t *pParamsOut) const;
		ISaveRestoreOps*		GetMaterialIndexDataOps() const;

	private:
		int						GetReservedSurfaceIndex(const char *pSurfacePropName) const;

		CSurface *				GetInternalSurface(int materialIndex);
		const CSurface *		GetInternalSurface(int materialIndex) const;

		void					CopyPhysicsProperties(CSurface *pOut, int baseIndex);

		unsigned short			FindString(const char *str) const;
		unsigned short			AddString(const char *str);
		bool					AddFileToDatabase(const char *pFilename);

	private:
		unsigned short m_idCounter;
		std::unordered_map<unsigned short, std::string> m_strings;
		std::vector<CSurface>	m_props;

		std::unordered_map<std::string, bool>	m_fileMap;
};

extern CPhysicsSurfaceProps g_SurfaceDatabase;

#endif // PHYSICS_SURFACEPROPS_H
