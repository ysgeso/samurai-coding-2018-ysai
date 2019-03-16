#pragma once

#include <cstring>
// 置換表に関するコード

// これを設定しておくと、置換表に関する様々なカウントデータを記録するようになる
#define TT_COUNT

// 置換表のキーの型。64ビットとする
using Key = uint64_t;
// 評価値。64ビットの浮動小数点数
using Hyouka = double;
constexpr Hyouka HYOUKA_INF = std::numeric_limits<double>::infinity();

// 置換表に記録するデータ（2のn乗のビット数にする）
struct TT_DATA {
	// キー（64ビット）
	Key			key;
	// 評価値（64ビット）
	Hyouka		hyouka;
};

// 置換表のハッシュのビット数-1
// キャッシュの関係上、17あたりが最速っぽい？
constexpr int TT_HASH_BIT = 17;
// クラスターの数
constexpr int TT_CLUSTERCOUNT = 1 << TT_HASH_BIT;
// クラスターのサイズ
constexpr int TT_CLUSTERSIZE = 4;
struct TT_CLUSTER {
	TT_DATA data[TT_CLUSTERSIZE];
};

struct alignas(32) TT_TABLE {
	TT_CLUSTER table[TT_CLUSTERCOUNT];
#ifdef TT_COUNT
	// この置換表に関する様々なカウントを行うための変数
	int hitcount, nothitcount, conflictcount, dropcount;
	int totalhitcount, totalnothitcount, totalconflictcount, totaldropcount;
#endif
	// key に一致するキャッシュがあればそれを返す
	TT_DATA *findcache(const Key key, bool& found) {
		TT_CLUSTER *c = &table[key & (TT_CLUSTERCOUNT - 1)];
		for (int i = 0; i < TT_CLUSTERSIZE; i++) {
			if (c->data[i].hyouka == 0) {
				break;
			}
			if (c->data[i].key == key) {
				found = true;
#ifdef TT_COUNT
				hitcount++;
				totalhitcount++;
				if (i > 0) {
					conflictcount++;
					totalconflictcount++;
				}
#endif
				return &c->data[i];
			}
		}
		found = false;
		for (int i = 0; i < 4; i++) {
			if (c->data[i].hyouka == 0) {
#ifdef TT_COUNT
				nothitcount++;
				totalnothitcount++;
#endif
				return &c->data[i];
			}
		}
		for (int i = TT_CLUSTERSIZE - 1; i > 0; i--) {
			c->data[i] = c->data[i - 1];
		}
#ifdef TT_COUNT
		dropcount++;
		totaldropcount++;
		nothitcount++;
		totalconflictcount++;
#endif
		return &c->data[0];
	}
#ifdef TT_COUNT
	TT_TABLE() : totalhitcount(0), totalnothitcount(0), totalconflictcount(0), totaldropcount(0) {
#else
	TT_TABLE() {
#endif
		clear();
	}
	void clear() {
#ifdef TT_COUNT
		hitcount = 0;
		nothitcount = 0;
		dropcount = 0;
		conflictcount = 0;
#endif
		memset(&table[0], 0, sizeof(TT_CLUSTER) * TT_CLUSTERCOUNT);
	}
#ifdef TT_COUNT
	void dump_count() const { 
		cerr << "hit " << hitcount << " nothit " << nothitcount << " confilct " << conflictcount << " drop " << dropcount << endl;
	}
	void dump_totalcount() const {
		cerr << "total hit " << totalhitcount << " nothit " << totalnothitcount << " confilct " << totalconflictcount << " drop " << totaldropcount << endl;
	}
#else
	void dump_count() const {}
	void dump_totalcount() const {}
#endif
	};