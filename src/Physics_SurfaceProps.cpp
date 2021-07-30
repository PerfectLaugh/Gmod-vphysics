#include "StdAfx.h"

#include <tier1/KeyValues.h>

#include "Physics_SurfaceProps.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/*****************************
* CLASS CPhysicsSurfaceProps
*****************************/

CPhysicsSurfaceProps::CPhysicsSurfaceProps() : m_idCounter(1) {
}

CPhysicsSurfaceProps::~CPhysicsSurfaceProps() {
}

int CPhysicsSurfaceProps::ParseSurfaceData(const char *pFilename, const char *pTextfile) {
	if (!AddFileToDatabase(pFilename)) return 0;
	DevMsg("VPhysics: Parsing surface data (file: %s)\n", pFilename);

	KeyValues *surfprops = new KeyValues("CPhysicsSurfaceProps");
	surfprops->LoadFromBuffer(pFilename, pTextfile);

	// Loop the outer elements (prop names with the data as subkeys)
	for (KeyValues *surface = surfprops; surface; surface = surface->GetNextKey()) {
		// Ignore duplicates
		if (GetSurfaceIndex(surface->GetName()) >= 0) {
			DevWarning("VPhysics: Ignoring duplicate surfaceprop \"%s\"\n", surface->GetName());
			continue;
		}

		CSurface prop;
		memset(&prop.data, 0, sizeof(prop.data));
		prop.m_name = AddString(surface->GetName());
		prop.data.game.material = 0;
		prop.data.game.maxSpeedFactor = 1.0f;
		prop.data.game.jumpFactor = 1.0f;
		prop.data.game.climbable = 0;

		int baseMaterial = GetSurfaceIndex("default");
		if (baseMaterial != -1) {
			CopyPhysicsProperties(&prop, baseMaterial);

			const CSurface *pSurface = GetInternalSurface(baseMaterial);
			prop.data.audio = pSurface->data.audio;
			prop.data.game = pSurface->data.game;
			prop.data.sounds = pSurface->data.sounds;
		}

		// Subkeys that contain the actual data
		for (KeyValues *data = surface->GetFirstSubKey(); data; data = data->GetNextKey()) {
			const char *key = data->GetName();
			if (!Q_stricmp(key, "base")) {
				baseMaterial = GetSurfaceIndex(data->GetString());
				CopyPhysicsProperties(&prop, baseMaterial);
			} else if (!Q_stricmp(key, "thickness"))
				prop.data.physics.thickness = data->GetFloat();
			else if (!Q_stricmp(key, "density"))
				prop.data.physics.density = data->GetFloat();
			else if (!Q_stricmp(key, "elasticity"))
				prop.data.physics.elasticity = data->GetFloat();
			else if (!Q_stricmp(key, "friction"))
				prop.data.physics.friction = data->GetFloat();
			else if (!Q_stricmp(key, "dampening"))
				prop.data.physics.dampening = data->GetFloat();
			else if (!Q_stricmp(key, "audioReflectivity"))
				prop.data.audio.reflectivity = data->GetFloat();
			else if (!Q_stricmp(key, "audioHardnessFactor"))
				prop.data.audio.hardnessFactor = data->GetFloat();
			else if (!Q_stricmp(key, "audioHardMinVelocity"))
				prop.data.audio.hardVelocityThreshold = data->GetFloat();
			else if (!Q_stricmp(key, "audioRoughnessFactor"))
				prop.data.audio.roughnessFactor = data->GetFloat();
			else if (!Q_stricmp(key, "scrapeRoughThreshold"))
				prop.data.audio.roughThreshold = data->GetFloat();
			else if (!Q_stricmp(key, "impactHardThreshold"))
				prop.data.audio.hardThreshold = data->GetFloat();
			else if (!Q_stricmp(key, "highPitchOcclusion"))
				prop.data.audio.highPitchOcclusion = data->GetFloat();
			else if (!Q_stricmp(key, "midPitchOcclusion"))
				prop.data.audio.midPitchOcclusion = data->GetFloat();
			else if (!Q_stricmp(key, "lowPitchOcclusion"))
				prop.data.audio.lowPitchOcclusion = data->GetFloat();
			else if (!Q_stricmp(key, "maxspeedfactor"))
				prop.data.game.maxSpeedFactor = data->GetFloat();
			else if (!Q_stricmp(key, "hidetargetid"))
				prop.data.game.hidetargetid = data->GetInt();
			else if (!Q_stricmp(key, "damageLossPercentPerPenetration"))
				prop.data.game.damageLossPercentPerPenetration = data->GetFloat();
			else if (!Q_stricmp(key, "jumpfactor"))
				prop.data.game.jumpFactor = data->GetFloat();
			else if (!Q_stricmp(key, "climbable"))
				prop.data.game.climbable = data->GetInt();
			else if (!Q_stricmp(key, "damagemodifier"))
				prop.data.game.damageModifier = data->GetInt();
			else if (!Q_stricmp(key, "penetrationmodifier"))
				prop.data.game.penetrationModifier = data->GetInt();
			else if (!Q_stricmp(key, "gamematerial")) {
				if (data->GetDataType() == KeyValues::TYPE_STRING && strlen(data->GetString()) == 1) {
					prop.data.game.material = toupper(data->GetString()[0]);
				} else {
					prop.data.game.material = data->GetInt();
				}
			} else if (!Q_stricmp(key, "stepleft")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.walkStepLeft = sym;
				prop.data.sounds.runStepLeft = sym;
			} else if (!Q_stricmp(key, "stepright")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.walkStepRight = sym;
				prop.data.sounds.runStepRight = sym;
			} else if (!Q_stricmp(key, "walkLeft")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.walkStepLeft = sym;
			} else if (!Q_stricmp(key, "walkRight")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.walkStepRight = sym;
			} else if (!Q_stricmp(key, "runLeft")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.runStepLeft = sym;
			} else if (!Q_stricmp(key, "runRight")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.runStepRight = sym;
			} else if (!Q_stricmp(key, "impactsoft")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.impactSoft = sym;
			} else if (!Q_stricmp(key, "impacthard")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.impactHard = sym;
			} else if (!Q_stricmp(key, "scrapesmooth")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.scrapeSmooth = sym;
			} else if (!Q_stricmp(key, "scraperough")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.scrapeRough = sym;
			} else if (!Q_stricmp(key, "bulletimpact")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.bulletImpact = sym;
			} else if (!Q_stricmp(key, "break")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.breakSound = sym;
			} else if (!Q_stricmp(key, "strain")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.strainSound = sym;
			} else if (!Q_stricmp(key, "rolling") || !Q_stricmp(key, "roll")) {
				auto sym = AddString(data->GetString());
				prop.data.sounds.rolling = sym;
			} else
				DevWarning("VPhysics: Surfaceprop \"%s\" has unknown key %s (data: %s)\n", surface->GetName(), key, data->GetString());
		}

		m_props.push_back(prop);
	}
	surfprops->deleteThis();
	return 0;
}

int CPhysicsSurfaceProps::SurfacePropCount() const {
	return m_props.size();
}

int CPhysicsSurfaceProps::GetSurfaceIndex(const char *pSurfacePropName) const {
	if (pSurfacePropName[0] == '$') {
		int index = GetReservedSurfaceIndex(pSurfacePropName);
		if (index >= 0) return index;
	}

	unsigned short id = FindString(pSurfacePropName);
	if (id != 0) {
		for (int i = 0; i < m_props.size(); i++) {
			if (m_props[i].m_name == id)
				return i;
		}
	}

	return -1;
}

void CPhysicsSurfaceProps::GetPhysicsProperties(int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity) const {
	const CSurface *pSurface = GetInternalSurface(surfaceDataIndex);
	if (pSurface) {
		if (friction) *friction		= pSurface->data.physics.friction;
		if (elasticity) *elasticity = pSurface->data.physics.elasticity;
		if (density) *density		= pSurface->data.physics.density;
		if (thickness) *thickness	= pSurface->data.physics.thickness;
	}
}

surfacedata_t *CPhysicsSurfaceProps::GetSurfaceData(int surfaceDataIndex) {
	CSurface *pSurface = GetInternalSurface(surfaceDataIndex);
	if (!pSurface) pSurface = GetInternalSurface(GetSurfaceIndex("default"));
	Assert(pSurface);

	return &pSurface->data;
}

const char *CPhysicsSurfaceProps::GetString(unsigned short stringTableIndex) const {
	return m_strings.at(stringTableIndex).c_str();
}

const char *CPhysicsSurfaceProps::GetPropName(int surfaceDataIndex) const {
	if (surfaceDataIndex < 0 || surfaceDataIndex > m_props.size())
		return "default";

	return m_strings.at(m_props[surfaceDataIndex].m_name).c_str();
}

void CPhysicsSurfaceProps::SetWorldMaterialIndexTable(int *pMapArray, int mapSize) {
	NOT_IMPLEMENTED
}

void CPhysicsSurfaceProps::GetPhysicsParameters(int surfaceDataIndex, surfacephysicsparams_t *pParamsOut) const {
	if (!pParamsOut) return;

	const CSurface *pSurface = GetInternalSurface(surfaceDataIndex);
	if (pSurface) {
		*pParamsOut = pSurface->data.physics;
	}
}

ISaveRestoreOps* CPhysicsSurfaceProps::GetMaterialIndexDataOps() const
{
	NOT_IMPLEMENTED

	return nullptr;
}

int CPhysicsSurfaceProps::GetReservedSurfaceIndex(const char *pSurfacePropName) const {
	if (!Q_stricmp(pSurfacePropName, "$MATERIAL_INDEX_SHADOW"))
		return MATERIAL_INDEX_SHADOW;
	
	return -1;
}

CSurface *CPhysicsSurfaceProps::GetInternalSurface(int materialIndex) {
	if (materialIndex < 0 || materialIndex > m_props.size()-1)
		return NULL;

	return &m_props[materialIndex];
}

const CSurface *CPhysicsSurfaceProps::GetInternalSurface(int materialIndex) const {
	if (materialIndex < 0 || materialIndex > m_props.size()-1)
		return NULL;

	return &m_props[materialIndex];
}

void CPhysicsSurfaceProps::CopyPhysicsProperties(CSurface *pOut, int baseIndex) {
	if (!pOut) return;

	const CSurface *pSurface = GetInternalSurface(baseIndex);
	if (pSurface) {
		pOut->data.physics = pSurface->data.physics;
	}
}

unsigned short CPhysicsSurfaceProps::FindString(const char *str) const {
	for (const auto &x : m_strings) {
		if (x.second == str) {
			return x.first;
		}
	}

	return 0;
}

unsigned short CPhysicsSurfaceProps::AddString(const char *str) {
	unsigned short id = FindString(str);

	if (id != 0) {
		return id;
	}

	id = m_idCounter++;
	m_strings[id] = std::string(str);

	return id;
}

bool CPhysicsSurfaceProps::AddFileToDatabase(const char *pFilename) {
	if (m_fileMap.count(pFilename) == 0) {
		m_fileMap[pFilename] = true;
		return true;
	}

	return false;
}

CPhysicsSurfaceProps g_SurfaceDatabase;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsSurfaceProps, IPhysicsSurfaceProps, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION, g_SurfaceDatabase);