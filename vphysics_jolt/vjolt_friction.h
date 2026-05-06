
#pragma once

#include <vector>

class JoltPhysicsObject;

class JoltPhysicsFrictionSnapshot final : public IPhysicsFrictionSnapshot
{
public:
	explicit JoltPhysicsFrictionSnapshot( JoltPhysicsObject *pObject );

	bool IsValid() override;

	IPhysicsObject *GetObject( int index ) override;
	int GetMaterial( int index ) override;

	void GetContactPoint( Vector &out ) override;

	void GetSurfaceNormal( Vector &out ) override;
	float GetNormalForce() override;
	float GetEnergyAbsorbed() override;

	void RecomputeFriction() override;
	void ClearFrictionForce() override;

	void MarkContactForDelete() override;
	void DeleteAllMarkedContacts( bool wakeObjects ) override;

	void NextFrictionData() override;
	float GetFrictionCoefficient() override;

	struct Contact
	{
		JoltPhysicsObject *pOther;
		Vector vNormal;
		Vector vContactPoint;
		float flPenetrationDepth;
	};

private:
	JoltPhysicsObject *m_pSelf = nullptr;
	std::vector<Contact> m_contacts;
	std::vector<JoltPhysicsObject *> m_DeleteList;
	size_t m_index = 0;
};
