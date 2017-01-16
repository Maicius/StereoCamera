#include "stdafx.h"
/**
*自顶向下解法
//*http://blog.csdn.net/chuhang_zhqr/article/details/52586793
*/
const int MAX_SIZE = 100;
long long dp[MAX_SIZE]={0};
long long DPUpStair(int n){
	if(dp[n])
		return dp[n];
	if(n==1)
		return 1;
	if(n==2)
		return 2;
	dp[n] = DPUpStair(n-1) + DPUpStair(n-2);
	return dp[n];
}