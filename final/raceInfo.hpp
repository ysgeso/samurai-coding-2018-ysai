#include <iostream>
#include <utility>
#include <list>
#include <chrono>
#include <vector>
#include <map>
#include <set>
#include <unordered_set>
#include <iomanip>
#include <cassert>
#include <string>
#include <random>
#include <fstream>
#include "simd.hpp"
#include "tt.hpp"

//#define DPRINT
//#define RECORDHIST

using namespace std;
using namespace rel_ops;


// 決勝に提出したもので、コメント以外はそのままです。
// MAX_LENGTH = 200, BB_NUM = 32 にしないと実際に決勝でおきたように、コース長が100を超えると不具合が生じます

// 予選のコースの条件
const int MAX_WIDTH = 20;
const int MAX_LENGTH = 100; // 決勝はこれを200にしなかったので・・・（涙
const int MIN_VISION = 5;
const int THINK_TIME_PER_STEP = 200;
const int THINK_TIME_MIN = 1000;
// 予選のコースから、盤面を表すビットボードの幅と高さ
const int BB_WIDTH = 32;
const int BB_HEIGHT = 8;
const int BB_NUM = 16; // (100 + 14(y軸方向の最大速度）+ 後方最大速度分(この場合14まで）) / (256 / 32)　（切り上げ）
                       // これも決勝は 200 で計算する必要あり。その場合はざっくりと、32にすればいいかと
// 予選のコースの条件から導き出される値
const int MIN_VEL_X = -5; // 横幅の最大が 20 なので、x方向の速度の絶対値の最大値は 5
const int MAX_VEL_X = 5;
const int MIN_VEL_Y = -5; // y軸方向の速度の最小値。大きく逆走することはありえなさそうなので、-5 としておく
const int MAX_VEL_Y = 14; // 長さの最大が 100 なので、y方向の速度の最大値は 14
						  // こちらは最大が200になっても、速度が15以上になることはまずなさそうなので、このままで大丈夫だと思われる（これを増やすと衝突判定のテーブルも増やす必要があるので変えたくない）

// 無限大
constexpr double D_INF = std::numeric_limits<double>::infinity();
constexpr int I_INF = std::numeric_limits<int>::infinity();

// 最大深さ
const int MAX_DEPTH = 32;

// 時間を計測するクラス
struct Timer {
	std::chrono::system_clock::time_point  start;
public:
	Timer() : start(std::chrono::system_clock::now()) {}

	void restart() {
		start = std::chrono::system_clock::now();
	}

	int time() {
		auto now = std::chrono::system_clock::now();  // 計測終了時間
		return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()); //処理に要した時間をミリ秒に変換
	}
};

// 整数の2次元ベクトルを表すクラス。ほぼ元のコードと同じ
struct IntVec {
	int x, y;

	// コンストラクタ
	IntVec(int x = 0, int y = 0) : x(x), y(y) {};
	// いくつかの演算子の定義
	IntVec operator+(const IntVec &another) {
		return IntVec(x + another.x, y + another.y);
	}
	bool operator==(const IntVec &another) const {
		return x == another.x && y == another.y;
	}
	bool operator<(const IntVec &another) const {
		return y == another.y ?	// If y position is the same
			x > another.x :		// better to be more to the left
		y < another.y;
	}
	void set(const int _x, const int _y) {
		x = _x;
		y = _y;
	}
	// ベクトルの長さを返す関数（追加）
	Hyouka length() const {
		return static_cast<Hyouka>(sqrt(x * x + y * y));
	}
};

ostream &operator<<(ostream &out, const IntVec &v);

// 位置、速度、加速を表す型の名前を設定（IntVecと同じ）
using Position = IntVec;
using Velocity = IntVec;
using Acceleration = IntVec;

// 盤面の一部の状態を表すビットボード
struct alignas(alignsize) BitBoard {
	// コンストラクタ。特に何もしない
	BitBoard() {}

	// 中身を 0 クリアする（dataは256ビット=32バイト）
	void clear() {
		memset(&data, 0, 32);
	}

	// 様々なビット演算に関する関数
	const BitBoard operator | (const BitBoard& bb) const {
		BitBoard retval;
		retval.data.m256i = OR256(data.m256i, bb.data.m256i);
		return retval;
	}
	const BitBoard& operator |= (const BitBoard& bb) {
		data.m256i = OR256(data.m256i, bb.data.m256i);
		return *this;
	}
	const BitBoard operator & (const BitBoard& bb) const {
		BitBoard retval;
		retval.data.m256i = AND256(data.m256i, bb.data.m256i);
		return retval;
	}
	const BitBoard& operator &= (const BitBoard& bb) {
		data.m256i = AND256(data.m256i, bb.data.m256i);
		return *this;
	}
	const BitBoard operator ^ (const BitBoard& bb) const {
		BitBoard retval;
		retval.data.m256i = XOR256(data.m256i, bb.data.m256i);
		return retval;
	}
	const BitBoard& operator ^= (const BitBoard& bb) {
		data.m256i = XOR256(data.m256i, bb.data.m256i);
		return *this;
	}
	const BitBoard operator ~() const {
		BitBoard retval;
#if defined(USE_AVX)
		retval.data.m256i = ANDNOT256(data.m256i, CMPEQ256(data.m256i, data.m256i));
#else
		retval.data = NOT256(data);
#endif
		return retval;
	}
	bool operator==(const BitBoard& bb) const {
		return (TESTC256(data.m256i, bb.data.m256i) & TESTC256(bb.data.m256i, data.m256i)) ? true : false;
	}
	bool operator!=(const BitBoard& bb) const {
		return (TESTC256(data.m256i, bb.data.m256i) & TESTC256(bb.data.m256i, data.m256i)) ? false : true;
	}
	// (~b1) & b2 を計算する 
	static BitBoard andnot(const BitBoard& b1, const BitBoard& b2) {
		BitBoard bb;
		bb.data.m256i = ANDNOT256(b1.data.m256i, b2.data.m256i);
		return bb;
	}
	// 自身が bb に完全に含まれているかどうかを調べる
	bool isincluded(const BitBoard& bb) const {
		return (*this & bb) == *this;
	}
	// すべてのビットが 0 かどうかを計算する
	bool iszero() const {
		return TESTC256(SETZERO256(), data.m256i) ? true : false;
	}
	void operator=(const BitBoard& bb) {
		memcpy(&data, &(bb.data), 32);
	}
	// (x, y) のビットを 1 にする
	void set(const int x, const int y) {
		assert(0 <= x && x < BB_WIDTH && y >= 0 && y < BB_HEIGHT);
		data.m32[y] |= 1 << x;
	}
	// p の位置のビットを 1 にする
	void set(const Position p) {
		set(p.x, p.y);
	}
	// (x, y) のビットを 0 にする
	void reset(const int x, const int y) {
		assert(0 <= x && x < BB_WIDTH && y >= 0 && y < BB_HEIGHT);
		data.m32[y] &= 0xffffffff - (1 << x);
	}
	// p のビットを 0 にする
	void reset(const Position p) {
		reset(p.x, p.y);
	}
	// (x, y) のビットが 1 の場合に true を返す
	bool isset(const int x, const int y) const {
		assert(0 <= x && x < BB_WIDTH && y >= 0 && y < BB_HEIGHT);
		return (data.m32[y] & (1 << x)) ? true : false;
	}
	// p のビットが 1 の場合に true を返す
	bool isset(Position p) const {
		return isset(p.x, p.y);
	}
	// ビットボードの内容をダンプする関数
	void dump() const {
		for (int y = BB_HEIGHT - 1; y >= 0; y--) {
			for (int x = 0; x < BB_WIDTH; x++) {
				cerr << (isset(x, y) ? "O" : "X");
			}
			cerr << endl;
		}
	}

	// 256ビットのデータ。BB_WIDTH * BB_HEIGHT 分のデータを表す
	M256 data;
};

// ビットボードを複数（SIZE）だけ集めたもの
template <unsigned SIZE> struct Board {
	// コンストラクタ。作成時に内容をすべて0クリアする
	Board() {
		clear();
	};
	// 中身をすべて0クリアする
	void clear() {
		for (auto& data: bb) {
			data.clear();
		}
	}
	// (x, y) の位置のビットを 1 にする関数
	void set(const int x, const int y) {
		assert(0 <= x && x < BB_WIDTH && y >= 0 && y < MAX_LENGTH + BB_HEIGHT);
		bb[y >> 3].set(x, y & 7);
	}
	// 上記の引数の型が Position バージョン
	void set(const Position p) {
		set(p.x, p.y);
	}
	// (x, y) の位置のビットを 0 にする関数
	void reset(const int x, const int y) {
		assert(0 <= x && x < BB_WIDTH && y >= 0 && y < MAX_LENGTH + BB_HEIGHT);
		bb[y >> 3].reset(x, y & 7);
	}
	void reset(const Position p) {
		reset(p.x, p.y);
	}
	// (x, y) の位置のビットが 1 かどうかをチェックする関数
	bool isset(const int x, const int y) const {
		return bb[y >> 3].isset(x, y & 7);
	}
	bool isset(const Position p) const {
		return isset(p.x, p.y);
	}
	// ダンプする関数
	void dump() {
		for (int i = SIZE - 1; i >= 0; i--) {
			cerr << "y: " << (i + 1) * BB_HEIGHT - 1 << endl;
			bb[i].dump();
		}
	}
	// ビットボードをSIZEだけ集めたもの
	BitBoard bb[SIZE];
};

// コースデータ
struct RaceCourse {
  uint64_t thinkTime;
  int stepLimit;
  int width, length;
  int vision;
  void dump() const {
	  cerr << "Course data. w:" << width << " l:" << length << " v:" << vision << " s:" << stepLimit << " t:" << thinkTime << endl;
  }
};

// 移動データ。元のAIで使っているので取ってあるが、自作AIでは使わない
struct Movement {
  Position from, to;
  list <Position> touchedSquares() const;
  Movement(Position from, Position to): from(from), to(to) {};
};

// プレーヤーの状態。元と比べて、加速などの情報も追加してある。
struct PlayerState {
  Position position;
  Velocity velocity;
  // 加速
  Acceleration acceleration;
  // ぶつかったか、妨害されたか、妨害したか、水上にいるか
  bool obstacled, intersected, isinwater;
  PlayerState() {}
  PlayerState(Position p, Velocity v) : position(p), velocity(v) {}
  PlayerState(Position p, Velocity v, Acceleration a) : position(p), velocity(v), acceleration(a) {}
  // ダンプする関数。endflag が true の場合は改行する
  void dump(const bool endlflag = true) const {
	  cerr << position << velocity << acceleration;
	  if (obstacled || intersected || isinwater) {
		  cerr << "(";
		  if (obstacled) {
			  cerr << "O";
		  }
		  if (intersected) {
			  cerr << "I";
		  }
		  if (isinwater) {
			  cerr << "W";
		  }
		  cerr << ")";
	  }
	  if (endlflag) {
		  cerr << endl;
	  }
  }
};

// マスの状況を表す
enum SquareInfo {
	Plain = 0,
	Obstacle = 1,
	Water = 2,
	Unknown = -1
};

// コースジェネレータが生成するいくつかのパターンを保持するデータ構造
struct MAPPATTERN {
	// 幅と長さ
	int width, length;
	// タイプ: 1: トラップ, ロゴ
	static const int TRAP = 1;
	static const int LOGO = 2;
	int type;
	// ロゴのデータかどうか
	bool islogo;
	// 各マスの情報
	char squares[MAX_LENGTH][MAX_WIDTH];
};

// 特定のターンのレース状況
struct RaceInfo {
	// コンストラクタ。初期設定を行う。後で constexpr にする？
	RaceInfo() {
		init();
	}

	// pos の位置から vel の速度で移動した場合、岩にぶつかるかどうか。
	// ただし、左右をはみ出すような移動はしないものとする。
#ifdef DPRINT
	bool isobstacled(const Position& pos, const Velocity& vel, bool dump = false) {
#else
	// dump が true の場合はデバッグ用にビットボードなどの情報をダンプする
	bool isobstacled(const Position& pos, const Velocity& vel) {
#endif	
		// ゴール方向に0または正の速度で加速する場合
		if (vel.y >= 0) {
			// 使用する obst のインデックス値を計算する（-pos.y を BB_HEIGHTで割った余り）
			int index = -pos.y % BB_HEIGHT;
			if (index < 0) {
				index += BB_HEIGHT;
			}
			// 使用する obst[index].bb のインデックス値を計算する pos.y を BB_HEIGHT で割った商
			int num = (pos.y + BB_HEIGHT - 1) / BB_HEIGHT;

#ifdef DPRINT
			if (dump) {
				cerr << "obst" << endl;
				obst->dump();
				cerr << "movetable " << pos.x << "," << vel.x - MIN_MOVE_TABLE_X << "," << vel.y << " index " << index << endl;
				movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].dump();
				cerr << "obst bb " << index << "," << num << endl;
				obst[index].dump();;
				cerr << "and " << endl;
				(movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb[0] & obst[index].bb[num]).dump();
				cerr << (movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb[0] & obst[index].bb[num]).iszero() << endl;
			}
#endif
			// y軸方向の速度が 8 以上の場合は、判定に2つBitboardが必要
			// movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb が移動の軌跡を表すビットボード
			// obst[index].bb[num] が障害物を表すビットボード
			// and を取って一つでもビットが立っていればぶつかると判定できる
			if (vel.y >= 8) {
				if ((movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb[0] & obst[index].bb[num]).iszero() &&
					(movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb[1] & obst[index].bb[num + 1]).iszero()) {
					return false;
				}
			}
			else {
				if ((movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb[0] & obst[index].bb[num]).iszero()) {
					return false;
				}
			}
		}
		// 速度が負の場合
		else {
			// y座標が負の場合は、負の座標に障害物は存在しないのでぶつからない
			if (pos.y < 0) {
				return false;
			}
			// 負の場合のindex, num の計算
			int index = (7 - pos.y) % BB_HEIGHT;
			if (index < 0) {
				index += BB_HEIGHT;
			}
			int num = ((pos.y - 7) + BB_HEIGHT - 1) / BB_HEIGHT;

#ifdef DPRINT
			if (dump) {
				cerr << "obst" << endl;
				obst->dump();
				cerr << "movetable " << pos.x << "," << vel.x - MIN_MOVE_TABLE_X << "," << vel.y << endl;
				movetable_minus[pos.x][vel.x - MIN_MOVE_TABLE_X][-vel.y].dump();
				cerr << "obst bb " << index << "," << num << endl;
				obst[index].dump();
				cerr << "and " << endl;
				(movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb[0] & obst[index].bb[num]).dump();
				cerr << (movetable_minus[pos.x][vel.x - MIN_MOVE_TABLE_X][-vel.y].bb[0] & obst[index].bb[num]).iszero() << endl;
			}
#endif
			if ((movetable_minus[pos.x][vel.x - MIN_MOVE_TABLE_X][-vel.y].bb[0] & obst[index].bb[num]).iszero()) {
				return false;
			}
		}
		return true;
	}

	// posからvelの速度での移動が、tposのタイルを通るかどうか
	bool is_movetile(const Position& pos, const Velocity& vel, const Position& tpos) {
		// 移動先の計算
		Position dpos(pos.x + vel.x, pos.y + vel.y);
		if ((tpos.x < pos.x && tpos.x < dpos.x) ||
			(tpos.x > pos.x && tpos.x > dpos.x) ||
			(tpos.y < pos.y && tpos.y < dpos.y) ||
			(tpos.y > pos.y && tpos.y > dpos.y)) {
			return false;
		}
		// tposとposのy座標の差
		int dy = tpos.y - pos.y;
		// ゴール方向に0または正の速度で加速する場合
		if (vel.y >= 0) {
			// 差が8未満の場合
			if (dy < 8) {
				return movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb[0].isset(tpos.x, dy);
			}
			// 8以上の場合
			// （y軸方向の速度の上限を MAX_VEL_Y = 14 としているので、dy >= 16 となることはありえない）
			else {
				return movetable[pos.x][vel.x - MIN_MOVE_TABLE_X][vel.y].bb[1].isset(tpos.x, dy - 8);
			}
		}
		else {
			return movetable_minus[pos.x][vel.x - MIN_MOVE_TABLE_X][-vel.y].bb[0].isset(tpos.x, 7 + dy);
		}
	}

	// RaceCourse の情報を元に、RaceInfo の width と length を設定する
	void setsize(const RaceCourse &course) {
		width = course.width;
		length = course.length;
	}

	// Position を unordered_set にそのまま入れるとハッシュが面倒なので、intに変換していれる。
	// その変換のための関数
	inline static int pos_to_int(const int x, const int y) {
		// 0 <= x < MAX_WIDTH = 20 なので、xは5ビットで表現できる
		return (y << 5) | x;
	}

	inline static int pos_to_int(const Position& p) {
		return pos_to_int(p.x, p.y);
	}

	inline static Position int_to_pos(const int num) {
		return Position(num & 0x1f, num >> 5);
	}

	// 各マスのゴールからの距離を計算する
	void calcdistance() {
		Timer t;

		// ゴールからの距離をインデックスとし、その距離の点のunordered_set をデータとする map
		// 未確定で、距離が無限大でない点をここに格納しておく
		// map なので距離の小さい順にソートされて格納される
		map<Hyouka, unordered_set<int>> poslist;

		// ダイクストラ法を使う
		// 初期化
		for (int y = 0; y < length; y++) {
			for (int x = 0; x < width; x++) {
				// 障害物でない場所に対して、
				if (squares[y][x] != SquareInfo::Obstacle) {
					// ゴールの一つ下の行の場合は、そこのゴールからの距離を１として、poslist[1]にその点を追加する
					if (y == length - 1) {
						dist[x][y] = 1;
						// 中央にいくほど評価値を高くしてみたが、弱くなるようなので下記は採用しない
						//if (x < (width - 1) / 2) {
						//	dist[x][y] += static_cast<Hyouka>(x) / 10000.0;
						//}
						//else {
						//	dist[x][y] += static_cast<Hyouka>(width - 1 - x) / 10000.0;
						//}
						poslist[dist[x][y]].insert(pos_to_int(x, y));
					}
					else {
						// そうでない場合は、距離を無限大とする。
						// なお、INF は数が多いので、poslist には入れない
						dist[x][y] = HYOUKA_INF;
					}
				}
			}
		}

		// poslist にデータが残っている間、処理を繰り返す
		while (poslist.size() > 0) {
			// 最も距離が小さいデータ を poslist から取り出す
			auto minposlist = poslist.begin();
			// その距離を distance に格納する
			Hyouka distance = minposlist->first;
			// その距離の各点について繰り返し処理を行う
			for (auto& pnum : minposlist->second) {
				// 点を整数値に変換する
				Position p = int_to_pos(pnum);
				// その点から、-dnumx <= dx <= dnumx, -dnumy <= dy <= dnumy の距離の全ての点に対して、処理を行う
				for (int dx = -dnumx; dx <= dnumx; dx++) {
					for (int dy = -dnumy; dy <= dnumy; dy++) {
						// (0, 0) の距離は移動してないので除く
						if (dx != 0 || dy != 0) {
							Velocity v(dx, dy);
							// dx, dy の距離の点
							Position np = v + p;
							// その整数値の表現
							int npnum = pos_to_int(np);
							// その点がゲーム盤の中にあり、その点への線分が障害物に阻まれない場合
							if (isinside(np) && !isobstacled(p, v)) {
								// 距離を計算し、もともとの距離より小さければ更新する
								Hyouka newdist = length_table[dx + dnumx][dy + dnumy] + distance;
								Hyouka currentdist = dist[np.x][np.y];
								if (currentdist > newdist) {
									// それまでの距離のデータをposlist から削除する。
									// ただし、-HYOUKA_INFはposlistに入れていないので消す必要はない
									if (currentdist != HYOUKA_INF) {
										poslist[currentdist].erase(npnum);
									}
									// 新しい距離としてposlistにその点を登録する
									poslist[newdist].insert(npnum);
									// 新しい距離で更新する
									dist[np.x][np.y] = newdist;
								}
							}
						}
					}
				}
			}
			// minposlist の点の距離は確定しているので、minposlist を poslist から削除する
			poslist.erase(minposlist);
		}

		cerr << "cd " << t.time() << "ms" << endl;
		totalcalcmovetime += t.time();
	}

	// p が 盤の中にあるかどうか
	bool isinside(const Position p) const {
		if (0 <= p.x && p.x < width && 0 <= p.y && p.y < length) {
			return true;
		}
		else {
			return false;
		}
	}

	// 速くならないのでこれは使わない
	// 点 p が 線分 p1-p2 の上にあるかどうかを判定する関数
	//bool is_on_segment_fast(const Position& p1, const Position& p2, const Position& p) {
	//	const int minx = min(p1.x, p2.x);
	//	if (p.x < minx) return false;
	//	const int maxx = max(p1.x, p2.x);
	//	if (p.x > maxx) return false;
	//	const int miny = min(p1.y, p2.y);
	//	if (p.y < miny) return false;
	//	const int maxy = max(p1.y, p2.y);
	//	if (p.y > maxy) return false;
	//	return is_on_segment_table[p2.x - p1.x - MIN_MOVE_TABLE_X][p2.y - p1.y - MIN_MOVE_TABLE_Y][p.x - p1.x - MIN_MOVE_TABLE_X][p.y - p1.y - MIN_MOVE_TABLE_Y];
	//}

	// 点 p が 線分 p1-p2 の上にあるかどうかを判定する関数
	bool is_on_segment(const Position& p1, const Position& p2, const Position& p) {
		const int minx = min(p1.x, p2.x);
		if (p.x < minx) return false;
		const int maxx = max(p1.x, p2.x);
		if (p.x > maxx) return false;
		const int miny = min(p1.y, p2.y);
		if (p.y < miny) return false;
		const int maxy = max(p1.y, p2.y);
		if (p.y > maxy) return false;
		int l1 = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
		int l2 = (p.x - p1.x) * (p.x - p1.x) + (p.y - p1.y) * (p.y - p1.y);
		if (l1 >= l2) {
			int l = (p2.x - p1.x) * (p.x - p1.x) + (p2.y - p1.y) * (p.y - p1.y);
			if (l >= 0 && l * l == l1 * l2) {
				return true;
			}
		}
		return false;
	}

	static bool is_intersect(const Position& p1, const Position& p2, const Position& e1, const Position &e2) {
		const int minpx = min(p1.x, p2.x);
		const int maxpx = max(p1.x, p2.x);
		const int minex = min(e1.x, e2.x);
		const int maxex = max(e1.x, e2.x);
		if (maxpx < minex || minpx > maxex) {
			return false;
		}
		const int minpy = min(p1.y, p2.y);
		const int maxpy = max(p1.y, p2.y);
		const int miney = min(e1.y, e2.y);
		const int maxey = max(e1.y, e2.y);
		if (maxpy < miney || minpy > maxey) {
			return false;
		}
		const int d1 = (p1.x - e1.x) * (e2.y - e1.y) - (p1.y - e1.y) * (e2.x - e1.x);
		const int d2 = (p2.x - e1.x) * (e2.y - e1.y) - (p2.y - e1.y) * (e2.x - e1.x);
		if (d1 * d2 > 0) {
			return false;
		}
		const int d3 = (e1.x - p1.x) * (p2.y - p1.y) - (e1.y - p1.y) * (p2.x - p1.x);
		const int d4 = (e2.x - p1.x) * (p2.y - p1.y) - (e2.y - p1.y) * (p2.x - p1.x);
		if (d3 * d4 > 0) {
			return false;
		}
		return true;
	}

	// ゴールからの距離を cerr にダンプする
	void dumpdist() const {
		for (int y = length - 1; y >= 0; y--) {
			cerr << setw(3) << y << ":";
			for (int x = 0; x < width; x++) {
				cerr << setw(5) << floor(dist[x][y] * 100) << " ";
			}
			cerr << endl;
		}
	}

	// zorbisthash 値を生成する
	Key create_hash(mt19937& mt, uniform_int_distribution<int>& rnd16) {
		Key k = 0;
		for (int i = 0; i < 4; i++) {
			k += static_cast<Key>(rnd16(mt));
			if (i != 3) {
				k <<= 16;
			}
		}
		return k;
	}
	
	// マップの岩とのの障害物との接触判定を高速化するためのテーブルの作成などの初期設定を行う
	void init() {
		Timer t;
		// 乱数関連
		// 乱数の種は固定しておく
		mt19937 mt(20190101);
		uniform_int_distribution<int> rnd16(0, (1 << 16) - 1);

		// 計算で使うベクトルの長さを計算するテーブルの作成
		for (int y = -dnumy; y <= dnumy; y++) {
			for (int x = -dnumx; x <= dnumx; x++) {
				length_table[x + dnumx][y + dnumy] = Velocity(x, y).length();
			}
		}

		// Zorbist ハッシュテーブルの初期化
		for (int y = 0; y < POS_TABLE_LENGTH; y++) {
			for (int x = 0; x < POS_TABLE_WIDTH; x++) {
				Zorbistmypos[x][y] = create_hash(mt, rnd16);
				Zorbistrvpos[x][y] = create_hash(mt, rnd16);
			}
		}
		for (int y = 0; y < VEL_TABLE_HEIGHT ; y++) {
			for (int x = 0; x < VEL_TABLE_WIDTH ; x++) {
				Zorbistmyvel[x][y] = create_hash(mt, rnd16);
				Zorbistrvvel[x][y] = create_hash(mt, rnd16);
			}
		}
		for (int d = 0; d < MAX_DEPTH; d++) {
			Zorbistdepth[d] = create_hash(mt, rnd16);
		}
		Zorbistenemyturn = create_hash(mt, rnd16);
		Zorbistintersected = create_hash(mt, rnd16);

		// ゲーム盤上での移動が、障害物に妨害されるかどうかを高速に計算するため、
		// あらかじめ想定されるすべてのケースについて、データを計算しておくことで高速に妨害判定を行う
		// そのためのデータの計算
		int ax, ay; // x,y 速度の絶対値
		int stepx, stepy;
		int x, y; // 現在のマスの座標
		double cx, cy; // 開始地点と終了地点を結ぶ線分と、x = 整数、または y = 整数 の直線の交点
		int sx; // 開始地点のマスのx座標
		int px, py; // cx を考慮した座標
		for (int vx = MIN_MOVE_TABLE_X; vx <= MAX_MOVE_TABLE_X; vx++) {
			for (int vy = MIN_MOVE_TABLE_Y; vy <= MAX_MOVE_TABLE_Y; vy++) {
				// 最初にいる地点は必ず登録する
				for (sx = 0; sx < MAX_WIDTH; sx++) {
					if (vy >= 0) {
						movetable[sx][vx - MIN_MOVE_TABLE_X][vy].set(sx, 0);
					}
					else {
						movetable_minus[sx][vx - MIN_MOVE_TABLE_X][-vy].set(sx, 7);
					}
				}
				if (vx > 0) {
					ax = vx;
					stepx = 1;
				}
				else {
					ax = -vx;
					stepx = -1;
				}
				if (vy > 0) {
					ay = vy;
					stepy = 1;
				}
				else {
					ay = -vy;
					stepy = -1;
				}
				// 開始地点と終了地点を結ぶ線分と、x=整数 の直線との交点を求め、その左右のマスを通る
				x = 0;
				while (x != vx) {
					cx = x + 0.5 + 0.5 * stepx;
					cy = vy * (cx - 0.5) / vx + 0.5;
					for (sx = 0; sx < MAX_WIDTH; sx++) {
						// 交点の左右のマスを通る
						for (int dx = -1; dx <= 0; dx++) {
							px = static_cast<int>(cx + sx + dx);
							py = static_cast<int>(floor(cy));
							if (px >= 0 && px < MAX_WIDTH) {
								if (vy >= 0) {
									movetable[sx][vx - MIN_MOVE_TABLE_X][vy].set(px, py);
								}
								else {
									movetable_minus[sx][vx - MIN_MOVE_TABLE_X][-vy].set(px, py - MIN_MOVE_TABLE_Y);
								}
								// 交点のy座標が整数の場合は、その一つ下のマスも通る
								if (cy == floor(cy)) {
									if (vy >= 0) {
										movetable[sx][vx - MIN_MOVE_TABLE_X][vy].set(px, py - 1);
									}
									else {
										movetable_minus[sx][vx - MIN_MOVE_TABLE_X][-vy].set(px, py - 1 - MIN_MOVE_TABLE_Y);
									}

								}
							}
						}
					}
					x += stepx;
				}
				// 開始地点と終了地点を結ぶ線分と、y=整数 の直線との交点を求め、その上下のマスを通る
				y = 0;
				while (y != vy) {
					cy = y + 0.5 + stepy * 0.5;
					cx = vx * (cy - 0.5) / vy + 0.5;
					for (sx = 0; sx < MAX_WIDTH; sx++) {
						// 交点の上下のマスを通る
						for (int dy = -1; dy <= 0; dy++) {
							py = static_cast<int>(cy + dy);
							px = static_cast<int>(floor(cx) + sx);
							if (px >= 0 && px < MAX_WIDTH) {
								if (vy >= 0) {
									movetable[sx][vx - MIN_MOVE_TABLE_X][vy].set(px, py);
								}
								else {
									movetable_minus[sx][vx - MIN_MOVE_TABLE_X][-vy].set(px, py - MIN_MOVE_TABLE_Y);
								}
							}
							// 交点のx座標が整数の場合は、その一つ左のマスも通る
							if (cx == floor(cx) && px - 1 >= 0 && px - 1 < MAX_WIDTH) {
								if (vy >= 0) {
									movetable[sx][vx - MIN_MOVE_TABLE_X][vy].set(px - 1, py);
								}
								else {
									movetable_minus[sx][vx - MIN_MOVE_TABLE_X][-vy].set(px - 1, py - MIN_MOVE_TABLE_Y);
								}
							}
						}
					}
					y += stepy;
				}
			}
		}

		//はやくならないので使わない
		//Position p0(0, 0);
		//// is_on_segment_table の生成
		//for (x = MIN_MOVE_TABLE_X; x <= MAX_MOVE_TABLE_X; x++) {
		//	for (y = MIN_MOVE_TABLE_Y; y <= MAX_MOVE_TABLE_Y; y++) {
		//		Position p1(x, y);
		//		for (int x2 = MIN_MOVE_TABLE_X; x2 <= MAX_MOVE_TABLE_X; x2++) {
		//			for (int y2 = MIN_MOVE_TABLE_Y; y2 <= MAX_MOVE_TABLE_Y; y2++) {
		//				Position p2(x2, y2);
		//				is_on_segment_table[x - MIN_MOVE_TABLE_X][y - MIN_MOVE_TABLE_Y][x2 - MIN_MOVE_TABLE_X][y2 - MIN_MOVE_TABLE_Y] = is_on_segment(p0, p1, p2);

		//			}
		//		}
		//	}
		//}

		// course_generator が生成する、決まったパターンのデータを記録したファイルから、データを読み込み、mappatterns に格納する
		ifstream ifs("data.txt");
		if (ifs.fail()) {
			cerr << "File do not exist.\n";
		}
		else {
			// 幅データの読み込み
			int w;
			ifs >> w;
			while (w >= 0) {
				MAPPATTERN mpat;
				mpat.width = w;
				// 長さ、トラップかどうかのデータの読み込み
				ifs >> mpat.length >> mpat.type;
				// マスのデータの読み込み
				for (y = 0; y < mpat.length; y++) {
					for (x = 0; x < mpat.width; x++) {
						int state;
						ifs >> state;
						mpat.squares[y][x] = static_cast<char>(state);
					}
				}
				mappatterns[w].push_back(mpat);
				// 次の幅データの読み込み。幅が負の場合は、そこで終了する
				ifs >> w;
			}
		}
		cerr << "init " << t.time() << "ms" << endl;
	}
	
	// AI_TYPE1で使用する、置換表のハッシュ値の計算を行う関数
	inline static int calc_hash_value(PlayerState& pl) {
		int hash = pl.position.x; // 0 ～ 20 の 5ビット
		hash = (hash << 7) + (pl.position.y + 128); // -128 ～ 127 の 8ビット
		hash = (hash << 4) + (pl.velocity.x + 5); // -5 ～ 5 の 4 ビット
		hash = (hash << 5) + (pl.velocity.y + 8); // -7 ～ 14 の 5 ビット　合計22ビット
		return hash;
	}

	// AI_TYPE2で使用する、置換表のハッシュ値の計算を行う関数
	// enemyturnがtrueの場合は、プレーヤーの動作後の状態と、敵の動作前の状態でハッシュ値を生成する。falseの場合と区別するため、Zorbistenemyturnを使用している
	inline Key calc_zorbist_hash(int depth, PlayerState& pl, PlayerState& en, bool enemyturn, bool intersected = false) {
		Key retval = Zorbistdepth[depth] ^ Zorbistmypos[pl.position.x][pl.position.y - POS_TABLE_MIN_Y] ^ Zorbistmyvel[pl.velocity.x - MIN_VEL_X][pl.velocity.y - MIN_VEL_Y] ^ Zorbistrvpos[en.position.x][en.position.y - POS_TABLE_MIN_Y] ^ Zorbistrvvel[en.velocity.x - MIN_VEL_X][en.velocity.y - MIN_VEL_Y];
		if (enemyturn) {
			retval ^= Zorbistenemyturn;
			if (intersected) {
				retval ^= Zorbistintersected;
			}
		}
		return retval;
	}

	// AI_TYPE2で使用する、置換表のハッシュ値の計算を行う関数。自分のみを考慮してハッシュ値を作る
	inline Key calc_zorbist_hash2(int depth, PlayerState& pl) {
		return Zorbistdepth[depth] ^ Zorbistmypos[pl.position.x][pl.position.y - POS_TABLE_MIN_Y] ^ Zorbistmyvel[pl.velocity.x - MIN_VEL_X][pl.velocity.y - MIN_VEL_Y];
	}

	// ターン開始時の初期化処理
	void turn_init() {
		// AITYPE1 で使用する置換表の初期化
		memset(reached, 0, sizeof(char) * REACHED_SIZE);
		memset(reached_depth, 0, sizeof(char) * REACHED_SIZE);
		memset(reached_value, 0, sizeof(Hyouka) * REACHED_SIZE);
		// AITYPE2 で使用する置換表の初期化
		tt_table.clear();
	}
	
	// ビットボードの内容をダンプする関数
	void coursedump() const {
		for (int y = length - 1; y >= 0; y--) {
			cerr << setw(3) << y << ":";
			for (int x = 0; x <width ; x++) {
				switch (squares[y][x]) {
				case SquareInfo::Plain:
					cerr << "O";
					break;
				case SquareInfo::Obstacle:
					cerr << "X";
					break;
				case SquareInfo::Water:
					cerr << "W";
					break;
				case SquareInfo::Unknown:
					cerr << "?";
					break;
				}
			}
			cerr << endl;
		}
	}

	// AITYPE2で使用する置換表
	TT_TABLE tt_table;

	vector<MAPPATTERN> mappatterns[MAX_WIDTH + 1];

	// calcmove に費やした総時間を格納する変数
	int totalcalcmovetime = 0;

	// お互いに邪魔しあって引き分けになるケースを阻止するために、
	// 前のターンと同じ位置、速度の場合に、前のターンと同じ行動を取らなくするようにするための変数
	PlayerState prevpl, prevene;
	// 前のターンと同じ位置、速度かどうか
	bool isprevsame;

	// move_table に関する様々な定義
	static const int MOVE_TABLE_SIZE = 2;
	static const int MOVE_TABLE_MINUS_SIZE = 1;
	static const int MIN_MOVE_TABLE_X = -5;
	static const int MAX_MOVE_TABLE_X = 5;
	static const int MIN_MOVE_TABLE_Y = -7;
	static const int MAX_MOVE_TABLE_Y = 14;
	static const int MOVE_TABLE_WIDTH = MAX_MOVE_TABLE_X - MIN_MOVE_TABLE_X + 1;
	static const int MOVE_TABLE_HEIGHT = MAX_MOVE_TABLE_Y + 1;
	static const int MOVE_TABLE_MINUS_HEIGHT = -MIN_MOVE_TABLE_Y + 1;
	static const int MOVE_TABLE_TOTAL_HEIGHT = MAX_MOVE_TABLE_Y - MIN_MOVE_TABLE_Y + 1;
	// 自分が (x, 0) にいた時に、vx, vy の速度で移動した場合に通るマスを表すビットボード
	// ただし、vy は0以上とする
	Board<MOVE_TABLE_SIZE> movetable[MAX_WIDTH][MOVE_TABLE_WIDTH][MOVE_TABLE_HEIGHT];
	// こちらは、vy が負の場合で 自分が (x, 7) にいた場合。vyの部分は正負を反転させて格納する。
	Board<MOVE_TABLE_MINUS_SIZE> movetable_minus[MAX_WIDTH][MOVE_TABLE_WIDTH][MOVE_TABLE_MINUS_HEIGHT];

	// n番目のインデックスの値を i_n とした場合
	// 点(i_2,i_3) が 線分 (0,0)-(i_0,i_1)上にあるかどうかを表すテーブル
	// 速くならないので使わない
	//bool is_on_segment_table[MOVE_TABLE_WIDTH][MOVE_TABLE_TOTAL_HEIGHT][MOVE_TABLE_WIDTH][MOVE_TABLE_TOTAL_HEIGHT];

	// AITYPE::TYPE1 で使用する
	static const int REACHED_SIZE = 1 << 22;
	// チェック済かどうかを表すテーブル
	char reached[REACHED_SIZE];
	// チェック済の場合の、探索深さを表すテーブル
	char reached_depth[REACHED_SIZE];
	// チェック済の場合の評価値を表すテーブル
	Hyouka reached_value[REACHED_SIZE];

	// AITYPE::TYPE2 で使用するハッシュ値を計算するためのテーブル
	static const int POS_TABLE_MIN_Y = -16;
	static const int POS_TABLE_MAX_Y = MAX_LENGTH + MAX_VEL_Y;
	static const int POS_TABLE_WIDTH = MAX_WIDTH;
	static const int POS_TABLE_LENGTH = POS_TABLE_MAX_Y - POS_TABLE_MIN_Y + 1;
	static const int VEL_TABLE_WIDTH = MAX_VEL_X - MIN_VEL_X + 1;
	static const int VEL_TABLE_HEIGHT = MAX_VEL_Y - MIN_VEL_Y + 1;
	Key Zorbistmypos[POS_TABLE_WIDTH][POS_TABLE_LENGTH];
	Key Zorbistrvpos[POS_TABLE_WIDTH][POS_TABLE_LENGTH];
	Key Zorbistmyvel[VEL_TABLE_WIDTH][VEL_TABLE_HEIGHT];
	Key Zorbistrvvel[VEL_TABLE_WIDTH][VEL_TABLE_HEIGHT];
	Key Zorbistdepth[MAX_DEPTH];
	Key Zorbistenemyturn, Zorbistintersected;

	// ゴールからの距離を計算する際に、各点から伸ばす線分のx,y方向の範囲の最大値の絶対値
	static constexpr int dnumx = 5;
	static constexpr int dnumy = 7;
	// 線分(0, 0) - (x, y)の距離。ただし、x, y の最小値は -dnumx, -dnumy なので、インデックスにはその値を足したものを入れる
	Hyouka length_table[dnumx * 2 + 1][dnumy * 2 + 1];

	// コースの幅、高さ
	int width, length;
	// 現在のターン数
	int stepNumber;
	// 残り時間
	uint64_t timeLeft;
	// 自分と敵の情報の配列。配列のインデックスは読みの深さ
	PlayerState me[MAX_DEPTH], opponent[MAX_DEPTH];
	// 各マスの状況。元のコードをそのまま流用。他と異なり、元のコードに合わせてインデックスは y, x の順となっている
	char squares[MAX_LENGTH][MAX_WIDTH];
	// ゴールからの距離。こちらは x, y の順とする
	Hyouka dist[MAX_WIDTH][MAX_LENGTH];
	// 盤面の障害物をビットボードで表現したもの。
	// インデックスの分だけ、一つy座標を下にずらして格納する（衝突計算の高速化が目的）
	Board<BB_NUM> obst[BB_HEIGHT];

	// 視界内の最大のy座標
	int sikaiy;
	// course_generator の特定のパターンをチェックするかどうか。その中のトラップのパターンをチェックするかどうか
	bool checkpattern, checktrap;
	// 上記が true の場合に、パターンが見つかったかどうか
	bool patfound;
	// 見つかったパターン
	MAPPATTERN pat;
	// 見つかったパターンが存在する一番下のy座標
	int paty;
	// ゴールまですべて視界内かどうか
	bool isallclear;
};

// AIの種類
enum AITYPE {
	ORIGINAL,   // オリジナル（サンプル）
	TYPE1,		// 自分のみ考慮するAI（初めにまず作ってみたもの）
	TYPE2,		// 敵も考慮するAI（本選ではこれを使う）
};

// AIで使用するパラメータ
struct Parameter {
	// AIの種類
	AITYPE aitype = AITYPE::TYPE2;
	// 元の障害物衝突判定を使うかどうか
	bool obstorig = false;
	// 読みの最大深さ
	int searchDepth = 10;
	// 初期の読みの最大深さ
	int searchDepthorig;
	// y方向の最小速度
	int minspeedy = -2;
	// y方向の最大速度
	int maxspeedy = 10;
	// 記録を取るかどうか
	bool recordhist = false;
	// 敵の読みの最大深さ
	int eneDepth = 3;
	// デバッグ表示を行うかどうか
	bool debugprint = false;
	// デバッグ表示を行うステップ数
	int debugstep = -1;
	// デバッグ表示を行う深さ
	int debugdepth = -1;
	// αβ枝刈を行うかどうか
	bool alphabeta = true;
	// 敵の評価値に乗算する係数
	Hyouka ehyoukamul[2] = { 1.0, 1.0 };
	// 自分がゴールしている場合に敵の評価値に乗算する係数
	Hyouka egoalhyoukamul[2] = { 1.0, 1.0 };
	// リーフノードで衝突して止まった場合の評価値
	Hyouka stophyouka[2] = { 0.5, 0.5 };
	// 敵の評価を、最小値ではない独自の評価で行う
	bool enehyoukaorig = false;
	// 独自の評価で使用する係数
	double emul[2] = { 2, 2 };
	// 評価値計算時に、深さ1あたりの点数
	Hyouka dmul[2] = { 3, 3 };
	// マップのパターンをチェックして補完するかどうか
	bool checkpattern = false;
	// マップのトラップのパターンをチェックするかどうか
	bool checktrap = false;
	// トラップのパターンをチェックした際に、評価値の低いほうに乗算する係数
	Hyouka mulmin[2] = { 1.0, 1.0 };
	// トラップのパターンをチェックした際に、評価値の高いほうに乗算する係数
	Hyouka mulmax[2] = { 1.0, 1.0 };
	// トラップのパターン時に、敵の行動を無視するかどうか
	bool ignoreenemyintrap = false;
	// ゴールの地形を考慮する距離
	int maxgoalpat_w9[2] = { 19, 19 };
	int maxgoalpat_w14[2] = { 15, 15 };
	int maxgoalpat_w19[2] = { 8, 8 };
	int maxgoalpat_w20[2] = { 0, 0 };
	bool goalcheck2 = false;

	// 先手 0 後手 1
	int teban = 0;

	bool is_debugprint(const RaceInfo& rinfo, const int depth) const {
		if (debugprint && (debugstep == -2 || debugstep == rinfo.stepNumber) && (debugdepth == -2 || depth <= debugdepth)) {
			return true;
		}
		return false;
	}
	// パラメータのパース
	void parseparam(int argc, char *argv[]) {
		// パラメータの解釈
		for (int i = 1; i < argc; i++) {
			if (strcmp(argv[i], "-d") == 0 && argc > i + 1) {
				i++;
				searchDepth = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-a") == 0 && argc > i + 1) {
				i++;
				switch (stoi(argv[i], NULL, 10)) {
				case 0:
					aitype = AITYPE::ORIGINAL;
					break;
				case 1:
					aitype = AITYPE::TYPE1;
					break;
				case 2:
					aitype = AITYPE::TYPE2;
					break;
				default:
					aitype = AITYPE::ORIGINAL;
					break;
				}
			}
			else if (strcmp(argv[i], "-o") == 0 && argc > i) {
				obstorig = true;
			}
			else if (strcmp(argv[i], "-h") == 0 && argc > i) {
				recordhist = true;
			}
			else if (strcmp(argv[i], "-nh") == 0 && argc > i) {
				recordhist = false;
			}
			else if (strcmp(argv[i], "-minsy") == 0 && argc > i + 1) {
				i++;
				minspeedy = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-maxsy") == 0 && argc > i + 1) {
				i++;
				maxspeedy = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-ed") == 0 && argc > i + 1) {
				i++;
				eneDepth = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-noab") == 0 && argc > i) {
				alphabeta = false;
			}
			else if (strcmp(argv[i], "-eo") == 0 && argc > i) {
				enehyoukaorig = true;
				alphabeta = false;
			}
			else if (strcmp(argv[i], "-emul") == 0 && argc > i + 2) {
				i++;
				emul[0] = stod(argv[i], NULL);
				i++;
				emul[1] = stod(argv[i], NULL);
				enehyoukaorig = true;
				alphabeta = false;
			}
			else if (strcmp(argv[i], "-ehmul") == 0 && argc > i + 2) {
				i++;
				ehyoukamul[0] = stod(argv[i], NULL);
				i++;
				ehyoukamul[1] = stod(argv[i], NULL);
			}
			else if (strcmp(argv[i], "-eghmul") == 0 && argc > i + 2) {
				i++;
				egoalhyoukamul[0] = stod(argv[i], NULL);
				i++;
				egoalhyoukamul[1] = stod(argv[i], NULL);
			}
			else if (strcmp(argv[i], "-sh") == 0 && argc > i + 2) {
				i++;
				stophyouka[0] = stod(argv[i], NULL);
				i++;
				stophyouka[1] = stod(argv[i], NULL);
			}
			else if (strcmp(argv[i], "-cp") == 0 && argc > i) {
				checkpattern = true;
			}
			else if (strcmp(argv[i], "-ct") == 0 && argc > i) {
				checktrap = true;
			}
			else if (strcmp(argv[i], "-mmin") == 0 && argc > i + 2) {
				i++;
				mulmin[0] = static_cast<Hyouka>(stod(argv[i], NULL));
				i++;
				mulmin[1] = static_cast<Hyouka>(stod(argv[i], NULL));
			}
			else if (strcmp(argv[i], "-mmax") == 0 && argc > i + 2) {
				i++;
				mulmax[0] = static_cast<Hyouka>(stod(argv[i], NULL));
				i++;
				mulmax[1] = static_cast<Hyouka>(stod(argv[i], NULL));
			}
			else if (strcmp(argv[i], "-ie") == 0 && argc > i) {
				ignoreenemyintrap = true;
			}
			else if (strcmp(argv[i], "-dmul") == 0 && argc > i + 2) {
				i++;
				dmul[0] = static_cast<Hyouka>(stod(argv[i], NULL));
				i++;
				dmul[1] = static_cast<Hyouka>(stod(argv[i], NULL));
			}
			else if (strcmp(argv[i], "-mgpw9") == 0 && argc > i + 2) {
				i++;
				maxgoalpat_w9[0] = stoi(argv[i], NULL, 10);
				i++;
				maxgoalpat_w9[1] = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-mgpw14") == 0 && argc > i + 2) {
				i++;
				maxgoalpat_w14[0] = stoi(argv[i], NULL, 10);
				i++;
				maxgoalpat_w14[1] = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-mgpw19") == 0 && argc > i + 2) {
				i++;
				maxgoalpat_w19[0] = stoi(argv[i], NULL, 10);
				i++;
				maxgoalpat_w19[1] = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-mgpw20") == 0 && argc > i + 2) {
				i++;
				maxgoalpat_w20[0] = stoi(argv[i], NULL, 10);
				i++;
				maxgoalpat_w20[1] = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-gc") == 0 && argc > i) {
				goalcheck2 = true;
			}
			else if (strcmp(argv[i], "-dp") == 0 && argc > i) {
				debugprint = true;
			}
			else if (strcmp(argv[i], "-dd") == 0 && argc > i + 1) {
				i++;
				debugdepth = stoi(argv[i], NULL, 10);
			}
			else if (strcmp(argv[i], "-ds") == 0 && argc > i + 1) {
				i++;
				debugstep = stoi(argv[i], NULL, 10);
			}
		}
		searchDepthorig = searchDepth;
//		dump();
	}

	// パラメータのダンプ
	void dump() {
		cerr << "Param: teban " << teban << " aitype " << aitype << " obstorig " << obstorig << " sdepth " << searchDepth << " enedepth " << eneDepth << endl;
		cerr << " recordhist " << recordhist << " minsy " << minspeedy << " maxsy " << maxspeedy << " ehyoukamul " << ehyoukamul[teban] << " egoalhyoukamul " << egoalhyoukamul[teban] << " stophyouka " << stophyouka[teban] << endl;
		cerr << " alphabeta " << alphabeta << " eorig " << enehyoukaorig << " emul " << emul[teban] << " dmul " << dmul[teban] << " checkpattern " << checkpattern  << " goalcheck2 " << goalcheck2 << endl;
		cerr << " checktrap " << checktrap << " mulmin " << mulmin[teban] << " mulmax " << mulmax[teban] <<  " ignoreenemy " << ignoreenemyintrap << " gpw9 " << maxgoalpat_w9[teban] << " gpw14 " << maxgoalpat_w14[teban] << " gpw19 " << maxgoalpat_w19[teban] << " gpw20 " << maxgoalpat_w20[teban] << endl;
 	}
};

// ターン毎のデータを記録するためのデータ構造
struct DATA {
	// myplan, myplan2 などが置換表を利用せずに呼ばれた回数と、総回数
	static constexpr int CNUM = 16;
	int count[CNUM], totalcount[CNUM];
	// ターンの思考時間と総時間
	uint64_t time, totaltime;
	// 残り時間
	uint64_t timeleft;
	// ターン開始時の時間
	uint64_t starttime;
	DATA() {
		clear();
	}
	void clear() {
		for (int i = 0; i < CNUM; i++) {
			count[i] = 0;
		}
	}
};

ostream &operator<<(ostream &out, const DATA &d);

// 各ターンのデータ
struct Turndata {
	// 総ターン数
	int turn;
	// 総思考時間
	uint64_t totaltime, maxtime;
	// 総 DATA の count
	int totalcount[DATA::CNUM];
	// 現在のターンのデータ
	DATA currentdata;
	// 各ターンのデータ
	vector<DATA> data;
	Turndata() : turn(0), totaltime(0), maxtime(0) {
		data.clear();
		currentdata.clear();
		for (int i = 0; i < DATA::CNUM; i++) {
			totalcount[i] = 0;
		}
		turn = 0;
	}
	// ターン開始時の処理
	void startturn(uint64_t time) {
		turn++;
		currentdata.clear();
		currentdata.starttime = time;
	}
	// 残り時間のセット
	void settimeleft(uint64_t time) {
		currentdata.timeleft = time;
	}
	// ターン終了時の処理
	void endturn(uint64_t time) {
		currentdata.time = time - currentdata.starttime;
		totaltime += currentdata.time;
		for (int i = 0; i < DATA::CNUM; i++) {
			totalcount[i] += currentdata.count[i];
			currentdata.totalcount[i] = totalcount[i];
		}
		currentdata.totaltime = totaltime;
		data.push_back(currentdata);
		if (currentdata.time > maxtime) {
			maxtime = currentdata.time;
		}
	}
	// currentdata.count を増やす
	void addcount(int num = 0) {
		currentdata.count[num]++;
	}
	void dumpcdata() const {
		cerr << "Turn " << turn << " " << currentdata;
	}
	void dumpall() const {
		cerr << "Turn Data " << turn << endl;
		for (int t = 0; t < turn - 1; t++) {
			cerr << "Turn " << setw(3) << right << t << " " << data[t] << endl;
		}
	}
};

// ゲームに関する情報をまとめたもの
struct GameData {
	// ベストな行動に対する最初の動作の加速を取得する関数
	Acceleration getbestacc() const {
		return bestmove[0][0].acceleration;
	}

	// ターン開始時の初期化処理
	void turn_init() {
		// 最善手の初期化
		bestmove[0][0].acceleration = Acceleration(0, 0);
		// 最善手の評価値の初期化（AITYPE1で使用。AITYPE2では使用しない）
		bestvalue = -100000;
		// rinfo の初期化
		rinfo.turn_init();
		
		// 最大速度を記録する
		Velocity& v = rinfo.me[0].velocity;
		if (v.x < minsx) {
			minsx = v.x;
		}
		if (v.x > maxsx) {
			maxsx = v.x;
		}
		if (v.y < minsy) {
			minsy = v.y;
		}
		if (v.y > maxsy) {
			maxsy = v.y;
		}
	}

	// ベストな行動をダンプする（αβを使わない場合はうまく記録されない・・・）
	void dumpbestmove() const {
		if (parameter.recordhist) {
			for (int i = 0; i <= parameter.searchDepth; i++) {
				cerr << i << ": ";
				bestmove[0][i].dump(false);
				enebestmove[0][i].dump(true);
			}
		}
	}

	// ダンプする
	void dump() {
		cerr << "speed xmin: " << minsx << " xmax: " << maxsx << " ymin: " << minsy << " ymax: " << maxsy << endl;
		cerr << "total calcmovetime " << rinfo.totalcalcmovetime << endl;
		if (parameter.aitype == AITYPE::TYPE2) {
			rinfo.tt_table.dump_totalcount();
		}
	}

	// コースの情報
	RaceCourse course;
	// 現在のターンのレースの情報
	RaceInfo rinfo;
	// ベストな行動の評価値（AITYPE1で使用）
	Hyouka bestvalue;
	// ベストな行動の読みの深さ（AITYPE1で使用）
	int bestdepth;
	// ベストな行動の各深さにおけるプレーヤーの状況（AITYPE2で使用。ただし、αβを使わない場合などではほぼ無効・・・）
	PlayerState bestmove[MAX_DEPTH][MAX_DEPTH];
	PlayerState enebestmove[MAX_DEPTH][MAX_DEPTH];
	// 深さ 0 のプレーヤーの行動に対する評価（trap地形を考慮する場合に使用）
	Hyouka depth0hyouka[3][3][2];
	// AIで使用するパラメータ
	Parameter parameter;
	// 各ターンのログデータ
	Turndata turndata;
	// 各方向のこれまでの最小、最大速度
	int minsx = I_INF;
	int maxsx = -I_INF;
	int minsy = I_INF;
	int maxsy = -I_INF;
};

istream &operator>>(istream &in, RaceCourse &course);
istream &operator>>(istream &in, PlayerState &ps);
istream &operator>>(istream &in, RaceInfo &ri);


