#include "Common.h"
#include "Matrix.h"
#include "Image.h"
#include "Reconstruct.h"
#include "sift.h"

void Test_1()
{
	Sift_Match_Map oMatch_Map;
	Sift_Match_Path("c:\\tmp\\temp", &oMatch_Map);
	Sift_Simple_Match_Item oMatch = oMatch_Map.m_pMatch[0];
	for (int i = 0; i < oMatch.m_iMatch_Count; i++)
	{
		printf("A:%f %f - B:%f %f\n", oMatch.m_pPoint_1[i][0], oMatch.m_pPoint_1[i][1],
			oMatch.m_pPoint_2[i][0], oMatch.m_pPoint_2[i][1]);
	}
	
	return;
}