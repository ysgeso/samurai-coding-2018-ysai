#include <algorithm>
#include <functional>
#include <queue>
#include <map>
#include <list>
#include <cctype>
#include <cmath>
#include "raceInfo.hpp"

// ゲームのデータを格納する変数の定義
GameData gamedata;
// ゲームのデータの各パラメータの参照の定義
RaceCourse& course = gamedata.course;
RaceInfo& rinfo = gamedata.rinfo;
Parameter& parameter = gamedata.parameter;
Turndata& tdata = gamedata.turndata;

// 元のAIのパラメータ
#ifndef SPEEDLIMIT
#define SPEEDLIMIT 1000
#endif
const int speedLimitSquared = SPEEDLIMIT * SPEEDLIMIT;

bool operator<(const PlayerState &ps1, const PlayerState &ps2) {
	return
		ps1.position == ps2.position ?
		ps1.velocity < ps2.velocity :
		ps1.position < ps2.position;
}

bool operator==(const PlayerState &ps1, const PlayerState &ps2) {
	return
		ps1.position == ps2.position &&
		ps1.velocity == ps2.velocity;
}

int nextSeq = 1;

struct Candidate {
	int seq;
	int step;			// Steps needed to come here
	PlayerState state;		// State of the player
	Candidate *from;		// Came here from this place
	Acceleration how;		//   with this acceleration
	bool goaled;
	double goalTime;		// Goal time, if goaled
	Candidate(int t, PlayerState s, Candidate *f, Acceleration a) :
		seq(nextSeq++), step(t), state(s), from(f), how(a) {
		goaled = state.position.y >= course.length;
		if (goaled) {
			goalTime = step +
				(course.length - state.position.y - 0.5) / state.velocity.y;
		}
	}
	bool operator<(const Candidate &c) const {
		return
			c.goaled ? !goaled || goalTime > c.goalTime :
		state == c.state ? step > c.step :
		state < c.state;
	}
};

ostream &operator<<(ostream &out, Candidate c) {
	out << "#" << c.seq << ": "
		<< c.step
		<< "@(" << c.state.position.x << "," << c.state.position.y << ")+("
		<< c.state.velocity.x << "," << c.state.velocity.y << ")"
		<< " <- #" << (c.from == nullptr ? 0 : c.from->seq);
	return out;
}

Acceleration plan(RaceInfo &info, const RaceCourse &course) {
		auto cmp = [](Candidate *a, Candidate *b) { return *a<*b; };
	priority_queue <Candidate *, vector<Candidate*>, decltype(cmp)>
		candidates(cmp);
	list <Candidate *> allocated;
	map <PlayerState, Candidate *> reached;
	PlayerState initial(info.me[0].position, info.me[0].velocity);
	Candidate initialCand(0, initial, nullptr, Acceleration(0, 0));
	reached[initial] = &initialCand;
	Candidate *best = &initialCand;
	candidates.push(&initialCand);
	do {
		tdata.addcount();
		Candidate *c = candidates.top();
		candidates.pop();
		for (int cay = 1; cay != -2; cay--) { // Try going forward first
			for (int cax = -1; cax != 2; cax++) {
				Acceleration accel(cax, cay);
				Velocity velo = c->state.velocity + accel;
				if (velo.x * velo.x + velo.y * velo.y <= speedLimitSquared) {
					Position pos = c->state.position + velo;
					if (0 <= pos.x && pos.x < course.width) {
						Movement move(c->state.position, pos);
						list <Position> touched = move.touchedSquares();
						if (pos != info.opponent[0].position &&
							none_of(touched.begin(), touched.end(),
								[info, course](Position s) {
							return
								0 <= s.y &&
								s.y < course.length &&
								info.squares[s.y][s.x] == 1;
						})) {
							if (0 <= pos.y && pos.y < course.length &&
								info.squares[pos.y][pos.x] == 2) {
								// Jumped into water
								velo = Velocity(0, 0);
							}
							PlayerState nextState(pos, velo);
							Candidate *nextCand =
								new Candidate(c->step + 1, nextState, c,
									Acceleration(cax, cay));
							allocated.push_front(nextCand);
							if (!nextCand->goaled &&
								c->step < parameter.searchDepth &&
								(reached.count(nextState) == 0 ||
									reached[nextState]->step > c->step + 1)) {
								candidates.push(nextCand);
								reached[nextState] = nextCand;
							}
							if (*nextCand > *best) {
								best = nextCand;
							}
						}
					}
				}
			}
		}
	} while (!candidates.empty());
	Acceleration bestAccel;
	if (best == &initialCand) {
		// No good move found
		// Slowing down for a while might be a good strategy
		int ax = 0, ay = 0;
		if (info.me[0].velocity.x < 0) ax += 1;
		else if (info.me[0].velocity.x > 0) ax -= 1;
		if (info.me[0].velocity.y < 0) ay += 1;
		else if (info.me[0].velocity.y > 0) ay -= 1;
		bestAccel = Acceleration(ax, ay);
	}
	else {
		Candidate *c = best;
		while (c->from != &initialCand) {
			c = c->from;
		}
		bestAccel = c->how;
	}
	for (Candidate *a : allocated) delete a;
	return bestAccel;
}

ostream &operator<<(ostream &out, const IntVec &v) {
	out << "(" << v.x << "," << v.y << ")";
	return out;
}

ostream &operator<<(ostream &out, const DATA &d) {
	out << "count ";
	for (int i = 1; i < 2; i++) {
		cerr << setw(6) << right << d.count[i] << "/" << setw(7) << right << d.totalcount[i] << " " ;
	}
	cerr << " time " << setw(6) << right << d.time << "/" << setw(8) << right << d.totaltime << " left" << setw(8) << right << d.timeleft;
	return out;
}

char *reached = gamedata.rinfo.reached;
char *reached_depth = gamedata.rinfo.reached_depth;
Hyouka *reached_value = gamedata.rinfo.reached_value;

// 敵を考慮しない古いバージョンのAI
Hyouka myplan(int depth) {
	PlayerState& pl = rinfo.me[depth];
	PlayerState& ene = rinfo.opponent[depth];

	int hash = rinfo.calc_hash_value(pl);
	if (reached[hash] && reached_depth[hash] <= depth) {
		return reached_value[hash];
	}

	// この関数が置換表を利用せずに呼ばれた回数
	tdata.addcount();
	// ゴールしているか、視界外に到達しているか、探索最大深さに到達しているかした場合は評価値を計算する。
	if (pl.position.y >= course.length || (pl.position.y >= 0 && rinfo.squares[pl.position.y][pl.position.x] == -1) || depth == parameter.searchDepth) {
		Hyouka value;
		if (pl.position.y >= course.length) {
			value = static_cast<Hyouka>((pl.position.y - course.length) * 100.0);
		}
		else if (pl.position.y < 0) {
			value = static_cast<Hyouka>(-rinfo.dist[pl.position.x][0] + pl.position.y * 100);
		}
		else {
			value = -rinfo.dist[pl.position.x][pl.position.y];
		}
		value += (parameter.searchDepth - depth) * 10000;

		if (gamedata.bestvalue < value) {
			gamedata.bestvalue = value;
			memcpy(&(gamedata.bestmove), &(rinfo.me), sizeof(PlayerState) * (depth + 1));
			memcpy(&(gamedata.enebestmove), &(rinfo.opponent), sizeof(PlayerState) * (depth + 1));
			gamedata.bestdepth = depth;
		}
		reached[hash] = 1;
		reached_value[hash] = value;
		reached_depth[hash] = static_cast<char>(depth);
		return value;
	}

	PlayerState& nextpl = rinfo.me[depth + 1];

	Hyouka maxvalue = -HYOUKA_INF;
	for (int cay = 1; cay != -2; cay--) { // Try going forward first
		for (int cax = -1; cax != 2; cax++) {
			Acceleration accel(cax, cay);
			Velocity velo = pl.velocity + accel;
			Position pos = pl.position + velo;
			if (velo.y < parameter.minspeedy || velo.y > parameter.maxspeedy) {
				continue;
			}
			bool obstacled = true;
			if (0 <= pos.x && pos.x < course.width) {
				Movement move(pl.position, pos);
				if (parameter.obstorig) {
					list <Position> touched = move.touchedSquares();
					if (!(depth == 0 && pos == ene.position) &&
						none_of(touched.begin(), touched.end(),
							[](Position s) {
						return
							0 <= s.y &&
							s.y < course.length &&
							rinfo.squares[s.y][s.x] == 1;
					})) {
						if (0 <= pos.y && pos.y < course.length &&
							rinfo.squares[pos.y][pos.x] == 2) {
							velo = Velocity(0, 0);
						}
						nextpl.position = pos;
						nextpl.velocity = velo;
						obstacled = false;
					}
				}
				else {
					if (!(depth == 0 && pos == ene.position) && !rinfo.isobstacled(pl.position, velo)) {
						if (0 <= pos.y && pos.y < course.length &&
							rinfo.squares[pos.y][pos.x] == 2) {
							velo = Velocity(0, 0);
						}
						nextpl.position = pos;
						nextpl.velocity = velo;
						obstacled = false;
					}
				}
			}
			if (obstacled) {
				nextpl.position = pl.position;
				nextpl.velocity.x = 0;
				nextpl.velocity.y = 0;
			}
			if (pl.velocity.x == 0 && pl.velocity.y == 0 && pl.position == nextpl.position.y) {
				continue;
			}
			pl.acceleration = accel;
			Hyouka value = myplan(depth + 1);
			if (maxvalue < value) {
				maxvalue = value;
			}
		}
	}
	reached[hash] = 1;
	reached_value[hash] = maxvalue;
	reached_depth[hash] = static_cast<char>(depth);
	return maxvalue;
}

// ここから実際のAIで使っているコード
// ps の動作を計算する。
// 制約に反する動作を行おうとした場合は false を返す
// 障害物などに妨害されて動けなかった場合は ps.obstacled を true に、そうでない場合は false にする
// checkenemy が true の場合は、es の位置を通過する動作は妨害されたものとする
// 妨害されなかった場合のみ ns に動作後の状態を設定する
 inline bool calc_movement(PlayerState& ps, PlayerState& ns) {
	// 速度の計算
	Velocity& velo = ns.velocity;
	velo = ps.velocity + ps.acceleration;
	// 制約を超えるような動きを行おうとした場合は false を返す
	if (velo.y < parameter.minspeedy || velo.y > parameter.maxspeedy) {
		return false;
	}
	// 次の場所の計算
	Position& pos = ns.position;
	pos = ps.position + velo;
	// 速度が0の場合は絶対に衝突しない
	if (velo.x == 0 && velo.y == 0) {
		ps.obstacled = false;
		return true;
	}

	ps.obstacled = true;
	// 左右の盤外に移動する場合は妨害される
	if (0 <= pos.x && pos.x < course.width) {
		// 敵の存在する点を通る場合は妨害される
		// そうでなければ、盤上の障害物との衝突チェックを行う
		if (!rinfo.isobstacled(ps.position, velo)) {
			// 妨害されなかった場合の処理
			// 水に突っ込んだ場合は速度を0にする
			if (0 <= pos.y && pos.y < course.length && rinfo.squares[pos.y][pos.x] == 2) {
				velo = Velocity(0, 0);
			}
			ps.obstacled = false;
		}
	}
	return true;
}
 
 // 敵を考慮しない評価値の計算
 // myplan から敵の部分を削ったもの。コメントは myplan2 を参照のこと
 Hyouka calc_hyouka(const int depth, bool isenemy) {
	 // 探索する depth における、プレーヤーへの参照
	 PlayerState* pl_p = isenemy ? rinfo.opponent : rinfo.me;
	 PlayerState* ene_p = isenemy ? rinfo.me : rinfo.opponent;

//	 PlayerState& pl = rinfo.me[depth];
//	 PlayerState& ene = rinfo.opponent[depth];
	 PlayerState& pl = pl_p[depth];
	 PlayerState& ene = ene_p[depth];

	 // プレーヤーの位置を元に、Zorbistハッシュ値を計算する
	 Key hash;
	 // 深さが0の場合のみ、敵の位置を考慮する
	 if (depth == 0 && ene.position.y < course.length) {
		 hash = rinfo.calc_zorbist_hash(depth, pl, ene, false);
	 }
	 else {
		 hash = rinfo.calc_zorbist_hash2(depth, pl);
	 }

	 bool found;
	 // 計算したハッシュ値から、置換表を探し存在した場合はその値を結果として返す。異なる局面でハッシュ値が一致する可能性は0ではないが、ハッシュ値は64ビットなので、その可能性は考慮しないことにする。
	 TT_DATA *ttdata = rinfo.tt_table.findcache(hash, found);
	 if (found) {
		 return ttdata->hyouka;
	 }
#ifdef RECORDHIST
	 if (parameter.recordhist) {
		 pl.isinwater = pl.position.y >= 0 && pl.position.y < course.length && rinfo.squares[pl.position.y][pl.position.x] == SquareInfo::Water;
	 }
#endif
	 // この関数が置換表を利用せずに呼ばれた回数
	 tdata.addcount();
	 // ゴールしているか、視界外に到達しているか、探索最大深さに到達しているかした場合は評価値を計算する。
	 if (pl.position.y >= course.length || (pl.position.y >= 0 && rinfo.squares[pl.position.y][pl.position.x] == SquareInfo::Unknown) || depth == parameter.searchDepth) {
		 Hyouka value;
		 // ゴールしている場合
		 if (pl.position.y >= course.length) {
			 // すべて視界内の場合は確実にゴールできる
			 if (rinfo.isallclear && depth > 0) {
				 // 一つ前の位置から、ゴールタイムを計算し、それをdepthに反映させる
				 int prevy = pl_p[depth - 1].position.y;
				 Hyouka goaldepth = static_cast<Hyouka>(depth) - static_cast<Hyouka>(pl.position.y - course.length) / static_cast<Hyouka>(pl.position.y - prevy);
				 value = (static_cast<Hyouka>(parameter.searchDepth) - goaldepth + 1.0) * 10000.0;

			 }
			 // そうでない場合はゴールできるとは限らない。評価値はゴールからの距離とする
			 // depth が searchDepth より小さい場合は、差分 * parameter.dmul を評価値に加える
			 else {
				 value = static_cast<Hyouka>(pl.position.y - course.length) + static_cast<Hyouka>(parameter.searchDepth - depth) * parameter.dmul[parameter.teban];
			 }
		 }
		 // スタート地点より手前の場合。(x,y) にいた場合、(x, 0) の評価値からの距離を引く
		 // depthがsearchDepth以下になることはないはず
		 else if (pl.position.y < 0) {
			 value = -rinfo.dist[pl.position.x][0] + static_cast<Hyouka>(pl.position.y);
		 }
		 // それ以外の場合
		 else {
			 // まず、その位置のゴールからの距離の負の値を評価値とする
			 // 速度に関しては速度が裏目に出る可能性もありそうなので評価に入れない
			 value = -rinfo.dist[pl.position.x][pl.position.y];
			 // リーフノードだった場合で、現在の速度が 0 で、前の位置と同じ位置で、前の速度が 0 でなければ、
			 // 衝突して止まったことにより、状況が進んでいるとみなして parameter.stophyouka を加える
			 if (parameter.searchDepth == depth && depth > 0) {
				 PlayerState& prevpl = pl_p[depth - 1];
				 if (pl.position == prevpl.position && pl.velocity.x == 0 && pl.velocity.y == 0 && !(prevpl.velocity == 0 && prevpl.velocity == 0)) {
					 value += parameter.stophyouka[parameter.teban];
				 }
			 }
			 // depth が searchDepth より小さい場合は、差分 * parameter.dmul を評価値に加える
			 else {
				 value += static_cast<Hyouka>(parameter.searchDepth - depth) * parameter.dmul[parameter.teban];
			 }
		 }

		 // 0の評価値は、置換表を使う際にまずいので、0にならないようにする
		 if (value == 0) {
			 value = static_cast<Hyouka>(0.000001);
		 }
#ifdef RECORDHIST
		 if (parameter.recordhist) {
			 gamedata.bestmove[depth][depth] = pl;
			 if (depth != parameter.searchDepth) {
				 gamedata.bestmove[depth][depth + 1].position.set(-100, -100);
			 }
		 }
#endif

		 ttdata->key = hash;
		 ttdata->hyouka = value;
		 return value;
	 }

	 // 次のプレーヤーへの参照
	 PlayerState& nextpl = pl_p[depth + 1];
	 // ぶつかった場合をチェック済かどうか
	 bool obst_checked = false;
	 Acceleration& pa = pl.acceleration;
	 Hyouka alpha = -HYOUKA_INF;
	 for (pa.y = 1; pa.y >= -1; pa.y--) {
		 for (pa.x = -1; pa.x <= 1; pa.x++) {
			 // 移動のチェック
			 tdata.addcount(1);
			 if (!calc_movement(pl, nextpl)) {
				 continue;
			 }
			 if (!pl.obstacled && depth == 0 && ene.position.y < course.length && rinfo.is_on_segment(pl.position, nextpl.position, ene.position)) {
				 pl.obstacled = true;
			 }
			 if (pl.obstacled) {
				 // 既にぶつかった場合をチェック済の場合は飛ばす
				 if (obst_checked) {
					 continue;
				 }
				 obst_checked = true;
				 nextpl.position = pl.position;
				 nextpl.velocity.set(0, 0);
			 }
			 // 現在の速度が 0 で、移動後の位置が変化しない場合は、何もしていないのと同じことなので、飛ばす
			 if (pl.velocity.x == 0 && pl.velocity.y == 0 && pl.position == nextpl.position) {
				 continue;
			 }

			 pl.intersected = false;
			 Hyouka hyouka = calc_hyouka(depth + 1, isenemy);

			 // 深さが 0 の場合のみ評価値を記録する
			 if (depth == 0) {
				 gamedata.depth0hyouka[pa.x + 1][pa.y + 1][0] = hyouka;
			 }

			 // α < 評価値 の場合、α を 評価値に更新する
			 if (alpha < hyouka) {
				 alpha = hyouka;
				 // bestmove の更新
				 if (depth == 0) {
					 gamedata.bestmove[depth][depth] = pl;
				 }
#ifdef RECORDHIST
				 if (parameter.recordhist) {
					 memcpy(&(gamedata.bestmove[depth][depth + 1]), &(gamedata.bestmove[depth + 1][depth + 1]), sizeof(PlayerState) * (parameter.searchDepth - depth + 1));
				 }
#endif
			 }
		 }
	 }
	 // 置換表に記録する
	 ttdata->key = hash;
	 ttdata->hyouka = alpha;
	 return alpha;
 }

 // AITYPE2 の場合の行動決定関数
Hyouka myplan2(const int depth, Hyouka alpha, Hyouka beta) {
	// 探索する depth における、プレーヤーと敵の情報への参照
	PlayerState& pl = rinfo.me[depth];
	PlayerState& ene = rinfo.opponent[depth];

#ifdef DPRINT
	if (parameter.is_debugprint(rinfo, depth)) {
		cerr << setw(depth) << "" << "myplan2 start depth " << depth << " alpha " << alpha << " beta " << beta << endl;
		pl.dump();
		ene.dump();
	}
#endif
	// 味方、敵の位置を元に、Zorbistハッシュ値を計算する
	Key hash = rinfo.calc_zorbist_hash(depth, pl, ene, false);

	bool found;
	// 計算したハッシュ値から、置換表を探し存在した場合はその値を結果として返す。
	// 異なる局面でハッシュ値が一致する可能性は0ではないが、ハッシュ値は64ビットなので、その可能性は考慮しないことにする。
	TT_DATA *ttdata = rinfo.tt_table.findcache(hash, found);
	if (found) {
		return ttdata->hyouka;
	}

#ifdef RECORDHIST
	if (parameter.recordhist) {
		pl.isinwater = pl.position.y >= 0 && pl.position.y < course.length && rinfo.squares[pl.position.y][pl.position.x] == SquareInfo::Water;
		ene.isinwater = ene.position.y >= 0 && ene.position.y < course.length && rinfo.squares[ene.position.y][ene.position.x] == SquareInfo::Water;
	}
#endif

	// この関数が置換表を利用せずに呼ばれた回数
	tdata.addcount();

	// 敵または味方が、ゴールしてるか、探索深さに達しているか、視界外に達しているかのいずれかの場合、
	// お互いの状況を無視することになるので、それぞれが単独で行動した場合の評価値を計算し、引き算したものを評価値として返す
	if (depth == parameter.searchDepth || depth == parameter.eneDepth ||
		pl.position.y >= course.length || (pl.position.y >= 0 && rinfo.squares[pl.position.y][pl.position.x] == SquareInfo::Unknown) ||
		ene.position.y >= course.length || (ene.position.y >= 0 && rinfo.squares[ene.position.y][ene.position.x] == SquareInfo::Unknown)) {
		// 敵の評価値を計算する
		Hyouka enehyouka = calc_hyouka(depth, true);
#ifdef RECORDHIST
		gamedata.enebestmove[depth][depth] = pl;
		//memcpy(&(gamedata.enebestmove[depth][depth]), &(gamedata.bestmove[depth][depth]), sizeof(PlayerState) * (parameter.searchDepth - depth + 1));
#endif
		// 自分の評価値を計算する。
		Hyouka plhyouka = calc_hyouka(depth, false);
		// 評価値を計算する
		Hyouka value;
		// 敵の評価が、ゴールしているものの場合は自分の評価値 - 敵の評価値 * parameter.egoalhyoukamul の係数
		if (enehyouka >= 10000) {
			value = plhyouka - enehyouka * parameter.egoalhyoukamul[parameter.teban];
		}
		else {
			// そうでなければ評価値は自分の評価値 - 敵の評価値 * parameter.ehyoukamul の係数
			value = plhyouka - enehyouka * parameter.ehyoukamul[parameter.teban];
		}


#ifdef DPRINT
		if (parameter.is_debugprint(rinfo, depth)) {
			cerr << setw(depth) << "" << "hyouka pl " << plhyouka << " en " << enehyouka  << endl;
		}
#endif
		// 置換表に記録する
		ttdata->key = hash;
		ttdata->hyouka = value;
		return value;
	}

	// 次の深さのプレーヤーと敵の情報への参照
	PlayerState& nextpl = rinfo.me[depth + 1];
	PlayerState& nextene = rinfo.opponent[depth + 1];

	// αβ法を使わない場合は alpha を -∞　とする
	if (!parameter.alphabeta) {
		alpha = -HYOUKA_INF;
	}

	// ぶつかった場合をチェック済かどうか
	bool obst_checked = false;
	// プレーヤーの加速への参照
	Acceleration& pa = pl.acceleration;
	// プレーヤーが状況が変化する行動を取らなかったかどうか
	//bool pl_noaction = false;
	// プレーヤーが妨害された場合をチェック済かどうか
	bool intersected_checked = false;
	// プレーヤーのとりうる各加速度について計算する
	for (pa.y = 1; pa.y >= -1; pa.y--) {
		for (pa.x = -1; pa.x <= 1; pa.x++) {
			// 敵に妨害されていないものとする
			pl.intersected = false;
			// このプレーヤーの行動に対する、敵の枝刈りされない行動が発見されていないかどうか。
			bool ene_notfound = true;
#ifdef DPRINT
			if (parameter.is_debugprint(rinfo, depth)) {
				cerr << setw(depth) << "" << "player acc " << pa << " depth " << depth << " alpha " << alpha << " beta " << beta << endl;
			}
#endif
			// 自身の行動を計算し、それが制約を満たさない行動であるか、障害物によって妨害されるかどうかをチェックする
			tdata.addcount(1);
			if (!calc_movement(pl, nextpl)) {
				// 制約を満たさない行動なので飛ばす
#ifdef DPRINT
				if (parameter.is_debugprint(rinfo, depth)) {
					cerr << setw(depth) << "" << "player invalid move. skip." << endl;
				}
#endif
				continue;
			}
#ifdef DPRINT
			if (parameter.is_debugprint(rinfo, depth)) {
				cerr << setw(depth) << "" << "player isobstacled " << pl.obstacled << endl;
			}
#endif
			// 障害物にぶつかる行動の場合
			if (pl.obstacled) {
				// 既にぶつかった場合をチェック済の場合は飛ばす
				if (obst_checked) {
#ifdef DPRINT
					if (parameter.is_debugprint(rinfo, depth)) {
						cerr << setw(depth) << "" << "player obstacled already checked. skip." << endl;
					}
#endif
					continue;
				}
				// 次の場所の位置と速度を修正する
				obst_checked = true;
				nextpl.position = pl.position;
				nextpl.velocity.set(0, 0);
			}
			// ぶつからない場合、敵の初期位置を通るかどうかをチェックする
			// 通る場合は必ず自分は敵に妨害されて移動できない
			// ただし、敵も同様に自分に妨害されて動けない可能性があるので、次の自分の位置と速度は変更しない
			else if (rinfo.is_on_segment(pl.position, nextpl.position, ene.position)) {
				if (intersected_checked) {
					continue;
				}
				intersected_checked = true;
				pl.intersected = true;
			}

#ifdef DPRINT
			if (parameter.is_debugprint(rinfo, depth)) {
				cerr << setw(depth) << "" << "player next." << endl;
				nextpl.dump();
			}
#endif
			// γ値（敵の行動の中で最も低い評価値を表す）に、敵の評価値の上限を表すβ値を代入する。
			Hyouka gamma = beta;
			// 敵と衝突した場合、プレーヤーの次の状態が変化する場合があるので、次の状態を取っておく
			PlayerState nextplbak = nextpl;
			// 敵の加速への参照
			Acceleration& ea = ene.acceleration;
			// 敵がぶつかった場合をチェック済かどうか
			bool ene_obst_checked = false;
			// 敵がプレーヤーによって妨害されたことをチェック済かどうか
			bool eintersect_checked = false;
			// 敵とプレーヤー双方が妨害されたことをチェック済かどうか
			bool bintersect_checked = false;
#ifdef RECORDHIST
			// プレーヤーと敵のベストな行動の記録
			PlayerState pbest[MAX_DEPTH];
			PlayerState ebest[MAX_DEPTH];
#endif
			// 敵の各行動の各評価値を格納するための変数。parameter.enehyoukaorig が true の場合に使う
			vector<Hyouka> ehyouka;

			// 次の味方、現在の敵の位置を元に、Zorbistハッシュ値を計算する(これほとんど効果なさそう）
			Key ehash;
			if (pl.intersected) {
				ehash = rinfo.calc_zorbist_hash(depth, pl, ene, true, true);
			}
			else {
				ehash = rinfo.calc_zorbist_hash(depth, nextpl, ene, true, false);
			}
			bool efound;
			TT_DATA *ettdata = rinfo.tt_table.findcache(ehash, efound);
			if (efound) {
				gamma = ettdata->hyouka;
				goto END_ENEMY_TURN;
			}
			// 敵の各行動に対する計算
			for (ea.y = 1; ea.y >= -1; ea.y--) {
				for (ea.x = -1; ea.x <= 1; ea.x++) {
#ifdef DPRINT
					if (parameter.is_debugprint(rinfo, depth)) {
						cerr << setw(depth) << "" << "enemy acc " << ea << endl;
					}
#endif
					// 妨害によって変化する可能性があるので、次のプレーヤーの状況を戻しておく
					nextpl = nextplbak;
					// 敵の行動を計算し、それが制約を満たさない行動であるか、障害物によって妨害されるかどうかをチェックする
					tdata.addcount(1);
					if (!calc_movement(ene, nextene)) {
						// 制約を満たさない行動なのでスキップする
#ifdef DPRINT
						if (parameter.is_debugprint(rinfo, depth)) {
							cerr << setw(depth) << "" << "enemy invalid move. skip." << endl;
						}
#endif
						continue;
					}

#ifdef DPRINT
					if (parameter.is_debugprint(rinfo, depth)) {
						cerr << setw(depth) << "" << "enemy obstacled " << ene.obstacled << endl;
					}
#endif

					// 敵の妨害状況をクリアしておく
					ene.intersected = false;

					// 障害物にぶつかる行動の場合
					if (ene.obstacled) {
						// 既にぶつかった場合をチェック済の場合は飛ばす
						if (ene_obst_checked) {
#ifdef DPRINT
							if (parameter.is_debugprint(rinfo, depth)) {
								cerr << setw(depth) << "" << "enemy obstacled already checked. skip." << endl;
							}
#endif
							continue;
						}
						ene_obst_checked = true;
						nextene.position = ene.position;
						nextene.velocity.set(0, 0);
					}

					// どちらも妨害されていない場合は、お互いの衝突判定を行う
					// （相手の初期位置に移動することによる衝突判定は、すでに実施済）
					if (!(pl.obstacled && ene.obstacled)) {
						// 衝突判定
						if (pl.intersected || RaceInfo::is_intersect(pl.position, nextpl.position, ene.position, nextene.position)) {
#ifdef DPRINT
							if (parameter.is_debugprint(rinfo, depth)) {
								cerr << setw(depth) << "" << "intersected " << endl;
							}
#endif
							// 相手の初期位置のタイルを通る場合は優先権を失う
							if (!pl.obstacled && !pl.intersected && rinfo.is_movetile(pl.position, nextpl.velocity, ene.position)) {
#ifdef DPRINT
								if (parameter.is_debugprint(rinfo, depth)) {
									cerr << setw(depth) << "" << "player is intesected. type 1." << endl;
								}
#endif
								pl.intersected = true;
							}
							if (!ene.obstacled && rinfo.is_movetile(ene.position, nextene.velocity, pl.position)) {
#ifdef DPRINT
								if (parameter.is_debugprint(rinfo, depth)) {
									cerr << setw(depth) << "" << "enemy is intersected. type 1" << endl;
								}
#endif
								ene.intersected = true;
							}
							// どちらも優先権を失っていない場合は、y座標が小さいほう、同じ場合は x 座標が小さいほうが優先権を得る
							if (!pl.intersected && !ene.intersected) {
								if (pl.position.y < ene.position.y || (pl.position.y == ene.position.y && pl.position.x < ene.position.x)) {
#ifdef DPRINT
									if (parameter.is_debugprint(rinfo, depth)) {
										cerr << setw(depth) << "" << "enemy is intesected. type 2." << endl;
									}
#endif
									ene.intersected = true;
								}
								else {
#ifdef DPRINT
									if (parameter.is_debugprint(rinfo, depth)) {
										cerr << setw(depth) << "" << "player is intesected. type 2." << endl;
									}
#endif
									pl.intersected = true;
								}
							}
							// すでに同じ状況の衝突チェックを行っているかどうかを調べ、チェック済の場合は飛ばす
							// プレーヤーが妨害された場合
							if (pl.intersected) {
								// 共に妨害された場合
								if (ene.intersected) {
									// すでにチェック済の場合は飛ばす
									if (bintersect_checked) {
#ifdef DPRINT
										if (parameter.is_debugprint(rinfo, depth)) {
											cerr << setw(depth) << "" << "both player intersected already checked. skip." << endl;
										}
#endif
										continue;
									}
									bintersect_checked = true;
									nextpl.position = pl.position;
									nextpl.velocity.set(0, 0);
									nextene.position = ene.position;
									nextene.velocity.set(0, 0);
								}
								// プレーヤーのみ妨害された場合
								else {
									nextpl.position = pl.position;
									nextpl.velocity.set(0, 0);
								}
							}
							// 敵のみが妨害された場合
							else if (ene.intersected) {
								// すでにチェック済の場合は飛ばす
								if (eintersect_checked) {
#ifdef DPRINT
									if (parameter.is_debugprint(rinfo, depth)) {
										cerr << setw(depth) << "" << "enemy intersected already checked. skip." << endl;
									}
#endif
									continue;
								}
								eintersect_checked = true;
								nextene.position = ene.position;
								nextene.velocity.set(0, 0);
							}
						}
					}

					// 深さ0で、前のターンと状況が同じで、自分と敵の位置が変化せず、敵の加速が前のターンと同じ場合は、同じ状況を繰り返すことになるので、プレーヤーがこの行動はとらないようにする
					if (depth == 0 && rinfo.isprevsame && pl.position == nextpl.position && ene.position == nextene.position) {
						//pl.dump();
						goto END_ENEMY_TURN_END;
					}

#ifdef DPRINT
					if (parameter.is_debugprint(rinfo, depth)) {
						cerr << setw(depth) << "" << "enemy plan2 start. depth  " << depth << endl;
						pl.dump();
						nextpl.dump();
						ene.dump();
						nextene.dump();
					}
#endif

					// 次の深さでこの関数を再起呼び出しする
					Hyouka value = myplan2(depth + 1, alpha, gamma);

#ifdef DPRINT
					if (parameter.is_debugprint(rinfo, depth)) {
						cerr << setw(depth) << "" << "enemy plan2 end. depth " << depth << " value " << value << " gamma " << gamma << endl;
						if (parameter.recordhist) {
							for (int i = 0; i < depth; i++) {
								cerr << i << ":";
								gamedata.bestmove[depth][i].dump(false);
								if (i <= parameter.eneDepth + 1) {
									gamedata.enebestmove[depth][i].dump(true);
								}
								else {
									cerr << endl;
								}
							}
						}
						cerr << depth << ":";
						pl.dump(false);
						ene.dump(true);
						if (parameter.recordhist) {
							for (int i = depth + 1; i <= parameter.searchDepth; i++) {
								cerr << i << ":";
								gamedata.bestmove[depth + 1][i].dump(false);
								if (i <= parameter.eneDepth + 1) {
									gamedata.enebestmove[depth + 1][i].dump(true);
								}
								else {
									cerr << endl;
								}
							}
						}
					}
#endif

					// 独自の評価の場合、評価値を記録して次へ（αβ枝刈は行わない）
					if (parameter.enehyoukaorig) {
						ehyouka.push_back(value);
						//if (rinfo.stepNumber == 26 && depth == 0) {
						//	cerr << pa << ea << value << endl;
						//}
						continue;
					}

					// 結果の評価値が γ 未満の場合、γ を評価値に更新する
					if (gamma > value) {
#ifdef DPRINT
						if (parameter.is_debugprint(rinfo, depth)) {
							cerr << setw(depth) << "" << "gamma updated. depth " << depth << " gamma " << gamma << " > " << value << endl;
						}
#endif
						gamma = value;
#ifdef RECORDHIST
						// 最善手の記録の更新
						if (parameter.recordhist) {
							pbest[depth] = pl;
							ebest[depth] = ene;
							memcpy(&(pbest[depth + 1]), &(gamedata.bestmove[depth + 1][depth + 1]), sizeof(PlayerState) * (parameter.searchDepth - depth));
							memcpy(&(ebest[depth + 1]), &(gamedata.enebestmove[depth + 1][depth + 1]), sizeof(PlayerState) * (parameter.searchDepth - depth));
						}
#endif
						// 敵の行動が見つかったので、ene_notfound を false にする
						ene_notfound = false;
					}

					// γ <= α（評価値の下限以下）の場合 γ = α として枝刈り
					if (parameter.alphabeta && alpha >= gamma) {
#ifdef DPRINT
						if (parameter.is_debugprint(rinfo, depth)) {
							cerr << setw(depth) << "" << "enemy alpha pr. " << alpha << "," << gamma << endl;
						}
#endif
						gamma = alpha;
						goto END_ENEMY_TURN;
					}
				}
			}

		END_ENEMY_TURN:
#ifdef DPRINT
			if (parameter.is_debugprint(rinfo, depth)) {
				cerr << setw(depth) << "" << "end enemy turn " << depth << " gamma " << gamma << " alpha " << alpha << " enenotfound " << ene_notfound<< endl;
				if (parameter.recordhist) {
					for (int i = 0; i < depth; i++) {
						cerr << i << ":";
						gamedata.bestmove[depth][i].dump(false);
						if (i <= parameter.eneDepth + 1) {
							gamedata.enebestmove[depth][i].dump(true);
						}
						else {
							cerr << endl;
						}
					}
				}
				cerr << depth << ":";
				pl.dump(false);
				ene.dump(true);
				if (parameter.recordhist) {
					for (int i = depth + 1; i <= parameter.searchDepth; i++) {
						cerr << i << ":";
						gamedata.bestmove[depth + 1][i].dump(false);
						if (i <= parameter.eneDepth + 1) {
							gamedata.enebestmove[depth + 1][i].dump(true);
						}
						else {
							cerr << endl;
						}
					}
				}
			}
#endif
			// 独自の評価の場合
			if (parameter.enehyoukaorig) {
				// 記録した評価値が存在する場合
				size_t size = ehyouka.size();
				if (size > 0) {
					// 評価値をソートする
					sort(ehyouka.begin(), ehyouka.end());
					gamma = 0;
					Hyouka div = 0;
					// 評価値はehyouka 内の値を大きい順に並べ替えたものを h_n とした場合 (parameter.emul ^ n * h_n の合計) / (parameter.emu; ^ n の合計）とする
					for (auto h : ehyouka) {
						// ∞の評価値は無視する
						if (h == HYOUKA_INF || h == -HYOUKA_INF) {
							continue;
						}
						gamma = static_cast<Hyouka>(gamma * parameter.emul[parameter.teban] + h);
						div = static_cast<Hyouka>(div * parameter.emul[parameter.teban] + 1);
					}
					// div が 0 の場合は飛ばす
					if (div == 0) {
						continue;
					}
					gamma /= div;
				}
				// 存在しない場合は、この行動を飛ばす
				else {
					continue;
				}
			}
			// 一つも敵の行動が採用されていない場合はそのままの値を採用する
			else if (ene_notfound) {
#ifdef DPRINT
				if (parameter.is_debugprint(rinfo, depth)) {
					cerr << setw(depth) << "" << "enemy not found. " << alpha << "," << gamma <<  endl;
				}
#endif
			}
			// 深さが 0 の場合は評価値を記録しておく（trap地形を考慮に入れた場合に使う）
			if (depth == 0) {
				gamedata.depth0hyouka[pa.x + 1][pa.y + 1][0] = gamma;
			}
			// 置換表に記録しておく
			ettdata->key = ehash;
			ettdata->hyouka = gamma;
			// α < γ の場合、評価値の下限を表す α を γに更新する
			if (alpha < gamma) {
#ifdef DPRINT
				if (parameter.is_debugprint(rinfo, depth)) {
					cerr << setw(depth) << "" << "alpha updated. depth " << depth << " alpha " << alpha << " > " << gamma << endl;
				}
#endif
				alpha = gamma;
#ifdef RECORDHIST
				if (parameter.recordhist) {
					memcpy(&(gamedata.bestmove[depth][depth]), &(pbest[depth]), sizeof(PlayerState) * (parameter.searchDepth - depth + 1));
					memcpy(&(gamedata.enebestmove[depth][depth]), &(ebest[depth]), sizeof(PlayerState) * (parameter.searchDepth - depth + 1));
				}
#endif
				if (depth == 0) {
					gamedata.bestmove[0][0] = pl;
				}
			}
			// α >= β の場合、この行動の評価値としてβを採用するという枝刈を行う
			if (parameter.alphabeta && alpha >= beta) {
				alpha = beta;
#ifdef RECORDHIST
				if (parameter.is_debugprint(rinfo, depth)) {
					cerr << setw(depth) << "" << "player beta pr." << endl;
				}
#endif
				goto END_TURN;
			}
		END_ENEMY_TURN_END:
			{}
		}
	}
END_TURN:
#ifdef DPRINT
	if (parameter.is_debugprint(rinfo, depth)) {
		cerr << setw(depth) << "" << "end turn " << depth << " alpha " << alpha << endl;
		if (parameter.recordhist) {
			gamedata.bestmove[depth][depth].dump();
			gamedata.bestmove[depth][depth + 1].dump();
			gamedata.enebestmove[depth][depth].dump();
			gamedata.enebestmove[depth][depth + 1].dump();
		}
	}
#endif
	// 置換表に記録する
	ttdata->key = hash;
	ttdata->hyouka = alpha;
	return alpha;
}

// RaceInfo の obstacled が正しく動作しているかどうか、元の判定と比べてチェックする関数
void checkobst(RaceCourse& rc, RaceInfo& rs) {
	for (int y = 0; y < 30; y++) {
		for (int x = 0; x < rc.width; x++) {
			Position p(x, y);
			for (int vx = -5; vx <= 5; vx++) {
				for (int vy = -7; vy <= 10; vy++) {
					Velocity v(vx, vy);
					Position pos = p + v;
					if (0 <= pos.x && pos.x < rc.width) {
						bool obst1 = true;
						bool obst2 = true;
						Movement move(p, pos);
						list <Position> touched = move.touchedSquares();
						if (none_of(touched.begin(), touched.end(),
							[rs, rc](Position s) {
							return
								0 <= s.y &&
								s.y < course.length &&
								rs.squares[s.y][s.x] == 1;
						})) {
							obst1 = false;
						}
						obst2 = rs.isobstacled(p, v);
						if (obst1 != obst2) {
							cerr << "(" << x << "," << y << ") (" << vx << "," << vy << ") " << obst1 << "," << obst2 << endl;
						}
					}
				}
			}

		}
	}
}

void benchmark() {
	constexpr int num = 10000000;
	{
		Timer t;
		mt19937 mt(20190101);
		uniform_int_distribution<int> rnd16(0, (1 << 16) - 1);
		for (int i = 0; i < num; i++) {
			int x = rnd16(mt) % course.width;
			int y = rnd16(mt) % course.length;
			if (rinfo.squares[y][x] == 1) {
				i--;
				continue;
			}
			int dx = rnd16(mt) % 11 - 5;
			int dy = rnd16(mt) % 10 - 2;
			Position p(x, y);
			Velocity v(dx, dy);
			Position p2 = p + v;
			//if (!(p2.x >= 0 && p2.x < course.width)) {
			//	i--;
			//}
		}
		cerr << "benchmark " << t.time() << "ms " << endl;
	}
	{
		Timer t;
		int c = 0;
		mt19937 mt(20190101);
		uniform_int_distribution<int> rnd16(0, (1 << 16) - 1);
		for (int i = 0; i < num; i++) {
			int x = rnd16(mt) % course.width;
			int y = rnd16(mt) % course.length;
			if (rinfo.squares[y][x] == 1) {
				i--;
				continue;
			}

			int dx = rnd16(mt) % 11 - 5;
			int dy = rnd16(mt) % 10 - 2;
			Position p(x, y);
			Velocity v(dx, dy);
			Position p2 = p + v;
			if (p2.x >= 0 && p2.x < course.width) {
				if (!rinfo.isobstacled(p, v)) {
					c++;
				}
			}
			//else {
			//	i--;
			//}
		}
		cerr << "benchmark " << t.time() << "ms " << " " << c << endl;
	}
	{
		Timer t;
		int c = 0;
		mt19937 mt(20190101);
		uniform_int_distribution<int> rnd16(0, (1 << 16) - 1);
		for (int i = 0; i < num; i++) {
			int x = rnd16(mt) % course.width;
			int y = rnd16(mt) % course.length;
			if (rinfo.squares[y][x] == 1) {
				i--;
				continue;
			}
			int dx = rnd16(mt) % 11 - 5;
			int dy = rnd16(mt) % 10 - 2;
			Position p(x, y);
			Velocity v(dx, dy);
			Position p2 = p + v;
			if (p2.x >= 0 && p2.x < course.width) {
				Movement move(p, p2);
				list <Position> touched = move.touchedSquares();
				if (none_of(touched.begin(), touched.end(),
					[](Position s) {
					return
						0 <= s.y &&
						s.y < course.length &&
						rinfo.squares[s.y][s.x] == 1;
				})) {
					c++;
				}
			}
			//else {
			//	i--;
			//}
		}
		cerr << "benchmark " << t.time() << "ms " << " " << c << endl;
	}
}

int main(int argc, char *argv[]) {
	// 時間を計測するための変数
	Timer timer;
	// パラメータを解釈する
	parameter.parseparam(argc, argv);
	// 一部のパラメータを rinfo にコピーする
	rinfo.checkpattern = parameter.checkpattern;
	rinfo.checktrap = parameter.checktrap;
	// 置換表のサイズのデバッグ表示
	cerr << "s tt " << sizeof(gamedata.rinfo.tt_table) / 1048576 << " ri " << sizeof(rinfo) / 1048576 << endl;
	// コースの読み込み
	cin >> course;
	// rinfo にゲーム盤のサイズを設定しておく
	rinfo.setsize(course);
	// コース情報のデバッグ表示
	course.dump();

	int slowcount = 0;
	cout << "0" << endl;
	cout.flush();
	while (!cin.eof()) {
		// ターン開始時の時間の記録
		tdata.startturn(timer.time());
		// データの読み込み
		cin >> rinfo;
		// 残り時間のセット
		tdata.settimeleft(rinfo.timeLeft);

		if (rinfo.stepNumber == 0) {
			if (rinfo.me[0].position.x < rinfo.opponent[0].position.x) {
				parameter.teban = 0;
			}
			else {
				parameter.teban = 1;
			}
			// emul, dmul, ehyoukamul, egoalhyoukamul, mulmax に関しては、コースジェネレータ―で作った100のコースで、自身の少し弱めのAIと対戦させて強化学習させた値を採用する
			// ただ、幅や視界が変化するとこれらの値が結構変化したので、下記のパラメータはおそらく過学習されたもので、本当に一般的に強いかどうかは不明
			// ただ、学習に結構時間がかかるので、幅は5,10,15,20、視界は5,10,20に対してのみしか学習できなかったので、下記のようにしている
			if (1) {
				if (parameter.teban == 0) {
					if (course.width < 10) {
						if (course.vision < 10) {
							parameter.emul[parameter.teban] = 2.375;
							parameter.dmul[parameter.teban] = 3.312;
							parameter.ehyoukamul[parameter.teban] = 2.5;
							parameter.egoalhyoukamul[parameter.teban] = 1;
							parameter.mulmax[parameter.teban] = 0.875;
						}
						else if (course.vision < 15) {
							parameter.emul[parameter.teban] = 1.5;
							parameter.dmul[parameter.teban] = 2.375;
							parameter.ehyoukamul[parameter.teban] = 0.875;
							parameter.egoalhyoukamul[parameter.teban] = 0.375;
							parameter.mulmax[parameter.teban] = 0.375;
						}
						else {
							parameter.emul[parameter.teban] = 2.25;
							parameter.dmul[parameter.teban] = 2;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 1;
							parameter.mulmax[parameter.teban] = 0.5;
						}
					}
					else if (course.width < 15) {
						if (course.vision < 10) {
							parameter.emul[parameter.teban] = 3.25;
							parameter.dmul[parameter.teban] = 2;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 1.25;
							parameter.mulmax[parameter.teban] = 1.5;
						}
						else if (course.vision < 15) {
							parameter.emul[parameter.teban] = 3.25;
							parameter.dmul[parameter.teban] = 1.25;
							parameter.ehyoukamul[parameter.teban] = 2.25;
							parameter.egoalhyoukamul[parameter.teban] = 1.75;
							parameter.mulmax[parameter.teban] = 0.5;
						}
						else {
							parameter.emul[parameter.teban] = 4.5;
							parameter.dmul[parameter.teban] = 4;
							parameter.ehyoukamul[parameter.teban] = 2;
							parameter.egoalhyoukamul[parameter.teban] = 2;
							parameter.mulmax[parameter.teban] = 1.5;
						}
					}
					else if (course.width < 20) {
						if (course.vision < 10) {
							parameter.emul[parameter.teban] = 2.5;
							parameter.dmul[parameter.teban] = 1.25;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 0.5;
							parameter.mulmax[parameter.teban] = 1.1;
						}
						else if (course.vision < 15) {
							parameter.emul[parameter.teban] = 2.25;
							parameter.dmul[parameter.teban] = 2.5;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 2;
							parameter.mulmax[parameter.teban] = 1.5;
						}
						else {
							parameter.emul[parameter.teban] = 3.75;
							parameter.dmul[parameter.teban] = 2.5;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 2;
							parameter.mulmax[parameter.teban] = 1.5;
						}
					}
					else {
						if (course.vision < 10) {
							parameter.emul[parameter.teban] = 2.5;
							parameter.dmul[parameter.teban] = 3.75;
							parameter.ehyoukamul[parameter.teban] = 1;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.1;
						}
						else if (course.vision < 15) {
							parameter.emul[parameter.teban] = 2.5;
							parameter.dmul[parameter.teban] = 3.75;
							parameter.ehyoukamul[parameter.teban] = 1;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.1;
						}
						else {
							parameter.emul[parameter.teban] = 2.5;
							parameter.dmul[parameter.teban] = 3.75;
							parameter.ehyoukamul[parameter.teban] = 1;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.1;
						}
					}
				}
				else {
					if (course.width < 10) {
						if (course.vision < 10) {
							parameter.emul[parameter.teban] = 3.625;
							parameter.dmul[parameter.teban] = 3.5;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 2;
						}
						else if (course.vision < 15) {
							parameter.emul[parameter.teban] = 2.75;
							parameter.dmul[parameter.teban] = 3.125;
							parameter.ehyoukamul[parameter.teban] = 2.5;
							parameter.egoalhyoukamul[parameter.teban] = 1;
							parameter.mulmax[parameter.teban] = 1.25;
						}
						else {
							parameter.emul[parameter.teban] = 4.5;
							parameter.dmul[parameter.teban] = 2.5;
							parameter.ehyoukamul[parameter.teban] = 2.5;
							parameter.egoalhyoukamul[parameter.teban] = 1;
							parameter.mulmax[parameter.teban] = 1.5;
						}
					}
					else if (course.width < 15) {
						if (course.vision < 10) {
							parameter.emul[parameter.teban] = 4.5;
							parameter.dmul[parameter.teban] = 1.25;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 1.75;
							parameter.mulmax[parameter.teban] = 2.125;
						}
						else if (course.vision < 15) {
							parameter.emul[parameter.teban] = 4.5;
							parameter.dmul[parameter.teban] = 2.5;
							parameter.ehyoukamul[parameter.teban] = 2;
							parameter.egoalhyoukamul[parameter.teban] = 2;
							parameter.mulmax[parameter.teban] = 1.5;
						}
						else {
							parameter.emul[parameter.teban] = 3.75;
							parameter.dmul[parameter.teban] = 2.5;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.1;
						}
					}
					else if (course.width < 20) {
						if (course.vision < 10) {
							parameter.emul[parameter.teban] = 3.25;
							parameter.dmul[parameter.teban] = 1.25;
							parameter.ehyoukamul[parameter.teban] = 1.5;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.25;
						}
						else if (course.vision < 15) {
							parameter.emul[parameter.teban] = 3.75;
							parameter.dmul[parameter.teban] = 3.25;
							parameter.ehyoukamul[parameter.teban] = 2;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.5;
						}
						else {
							parameter.emul[parameter.teban] = 2.25;
							parameter.dmul[parameter.teban] = 4;
							parameter.ehyoukamul[parameter.teban] = 2.5;
							parameter.egoalhyoukamul[parameter.teban] = 2;
							parameter.mulmax[parameter.teban] = 1.5;
						}
					}
					else {
						if (course.vision < 10) {
							parameter.emul[parameter.teban] = 2.5;
							parameter.dmul[parameter.teban] = 2.5;
							parameter.ehyoukamul[parameter.teban] = 1;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.1;
						}
						else if (course.vision < 15) {
							parameter.emul[parameter.teban] = 2.5;
							parameter.dmul[parameter.teban] = 2.5;
							parameter.ehyoukamul[parameter.teban] = 1;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.1;
						}
						else {
							parameter.emul[parameter.teban] = 2.5;
							parameter.dmul[parameter.teban] = 2.5;
							parameter.ehyoukamul[parameter.teban] = 1;
							parameter.egoalhyoukamul[parameter.teban] = 1.5;
							parameter.mulmax[parameter.teban] = 1.1;
						}
					}
				}
			}
			parameter.dump();
		}

		//if (rinfo.stepNumber == 25) {
		//	benchmark();
		//}
		// 特定のステップで障害物と距離をデバッグ表示するコード
		if (rinfo.stepNumber == 42) {
//			rinfo.obst[0].dump();
		//	rinfo.dumpdist();
			//rinfo.coursedump();
		}

		// 終了判定（official.exe に独自の改造を行い、ゲーム終了後に -1 が返るようにしてある）
		if (rinfo.stepNumber < 0) {
			break;
		}

		// 最初は逆走したりする場合もあるので、y座標とコースに10を足したもので計算する
		double goaltime = (course.length + 10) * rinfo.stepNumber / (rinfo.me[0].position.y + 10);
		bool slowdown = false;
		// ゴールタイムを 仮定し、足りそうな場合は searchDepth を 1 減らす
		if (rinfo.me[0].position.y > -10) {
			if (parameter.searchDepth > 5 && rinfo.timeLeft < course.thinkTime * (goaltime - rinfo.stepNumber) / goaltime / 4) {
				slowdown = true;
			}
			if (parameter.searchDepth > 5 && rinfo.timeLeft < course.thinkTime * (goaltime - rinfo.stepNumber) / goaltime / 2) {
				parameter.searchDepth -= 2;
				slowcount++;
			}
			else if (parameter.searchDepth > 5 && rinfo.timeLeft < course.thinkTime * (goaltime - rinfo.stepNumber) / goaltime) {
				parameter.searchDepth--;
				slowcount++;
			}
			else if (parameter.searchDepth < parameter.searchDepthorig && rinfo.timeLeft > course.thinkTime * (goaltime - rinfo.stepNumber) / goaltime) {
				parameter.searchDepth += 3;
				if (parameter.searchDepth > parameter.searchDepthorig) {
					parameter.searchDepth = parameter.searchDepthorig;
				}
			}
		}

		// ステップ数のデバッグ表示
		cerr << "s:" << rinfo.stepNumber << " " << rinfo.sikaiy << " " << parameter.searchDepth << "/" << parameter.searchDepthorig << endl;

		// 出力する加速度の値を格納する変数
		Acceleration accel;
		// AIの種類によって処理を変える
		switch (parameter.aitype) {
		case AITYPE::ORIGINAL:
			// ほぼ元のAIと同じ処理
			accel = plan(rinfo, course);
			break;
		case AITYPE::TYPE1:
		case AITYPE::TYPE2:
			// ターン開始時の初期設定処理
			gamedata.turn_init();
			// 評価値
			Hyouka value;
			// type1 の AIの処理
			if (parameter.aitype == AITYPE::TYPE1) {
				value = myplan(0);
			}
			// type2 の AI
			else {
				// 深さ0の各行動の評価値を最低値に設定しておく
				for (int x = -1; x <= 1; x++) {
					for (int y = -1; y <= 1; y++) {
						gamedata.depth0hyouka[x + 1][y + 1][0] = -HYOUKA_INF;
						gamedata.depth0hyouka[x + 1][y + 1][1] = -HYOUKA_INF;
					}
				}
				// 視界外のy座標の幅
				int unknowny = course.length - rinfo.sikaiy - 1;

				// 視界内の最後の地形がトラップ地形の一部だった場合の処理
				// checkpattern, checktrap が true の場合のみ処理を行う
				if (parameter.checkpattern && parameter.checktrap && rinfo.patfound && rinfo.pat.type == MAPPATTERN::TRAP) {
					// トラップの評価を行ったことを表すデバッグ表示
					cerr << "th" << endl;
					// 敵の行動を無視しない場合
					if (!parameter.ignoreenemyintrap) {
						// 通常の myplan2 で行動を決定する
						myplan2(0, -HYOUKA_INF, HYOUKA_INF);
					}
					// 敵の行動を無視する場合
					else {
						// 敵の行動を無視して行動を決定する
						calc_hyouka(0, false);
					}
					// プレーヤーの各行動に対する評価値をコピーする
					for (int y = -1; y <= 1; y++) {
						for (int x = -1; x <= 1; x++) {
							gamedata.depth0hyouka[x + 1][y + 1][1] = gamedata.depth0hyouka[x + 1][y + 1][0];
						}
					}
					// ターン開始時の初期処理をもう一度行う（置換表のクリア）
					gamedata.turn_init();
					MAPPATTERN& pat = rinfo.pat;
					// トラップの部分の地形を反転させ、障害物と地形データに反映させる
					// 異なるのは最後の行だけなので、その行だけを反映する
					int y = rinfo.paty + pat.length - 1;
					for (int x = 0; x < course.width; x++) {
						rinfo.squares[y][x] = rinfo.pat.squares[pat.length - 1][course.width - x - 1];
						// 障害物だった場合は、障害物のビットボードを設定
						// ビットボード1つの高さの数だけ、y座標を一つずつずらしたものを設定する
						for (int i = 0; i < BB_HEIGHT; i++) {
							if (rinfo.squares[y][x] == SquareInfo::Obstacle) {
								rinfo.obst[i].set(x, y + i);
							}
							else {
								rinfo.obst[i].reset(x, y + i);
							}
						}
					}
					// 各マスのゴールからの距離を再計算する
					rinfo.calcdistance();

					// もう一度計算を行う
					if (!parameter.ignoreenemyintrap) {
						myplan2(0, -HYOUKA_INF, HYOUKA_INF);
					}
					else {
						calc_hyouka(0, false);
					}
					// 評価値の計算
					Hyouka maxhyouka = -HYOUKA_INF;
					Hyouka hyouka;
					// どちらを採用したかを数えるための変数（デバッグ用）
					//Hyouka c1 = 0;
					//Hyouka c2 = 0;
					// 評価値は各行動に対して、小さいほうに mulmin, 大きいほうに mulmax を乗算したものを加算し、(mulmin + mulmax)で割った値とする
					// その中の最大値を取るものを採用する
					for (y = -1; y <= 1; y++) {
						for (int x = -1; x <= 1; x++) {
							Hyouka m0, m1;
							if (gamedata.depth0hyouka[x + 1][y + 1][0] < gamedata.depth0hyouka[x + 1][y + 1][1]) {
								m0 = parameter.mulmin[parameter.teban];
								m1 = parameter.mulmax[parameter.teban];
								//c1++;
							}
							else {
								m0 = parameter.mulmax[parameter.teban];
								m1 = parameter.mulmin[parameter.teban];
								//c2++;
							}
							hyouka = (gamedata.depth0hyouka[x + 1][y + 1][0] * m0 + gamedata.depth0hyouka[x + 1][y + 1][1] * m1) / (m0 + m1);
							if (hyouka > maxhyouka) {
								maxhyouka = hyouka;
								// bestaccの 更新
								gamedata.bestmove[0][0].acceleration.set(x, y);
							}
						}
					}
					value = maxhyouka;
				}
				// 幅が10未満、15未満、20未満、20それぞれについて、ゴール前の予測を行う
				else if (!slowdown && parameter.goalcheck2 && parameter.checkpattern && unknowny > 0 &&
					((unknowny <= parameter.maxgoalpat_w9[parameter.teban] && course.width < 10) ||
					(unknowny <= parameter.maxgoalpat_w14[parameter.teban] && 10 <= course.width && course.width < 15) ||
						(unknowny <= parameter.maxgoalpat_w19[parameter.teban] && 15 <= course.width && course.width < 20) ||
						(unknowny >= 9 && unknowny <= parameter.maxgoalpat_w20[parameter.teban] && 15 <= course.width && course.width == 20))) {
					// ゴールに関する評価を行ったことを表すデバッグ表示
					cerr << "cg2" << endl;
					// 繰り返すので、足りなさそうな場合は searchDepth を一気に減らす
					if (rinfo.me[0].position.y > -10) {
						if (parameter.searchDepth > 5 && rinfo.timeLeft < course.thinkTime * (goaltime - rinfo.stepNumber) / goaltime / 2) {
							parameter.searchDepth = 5;
							slowcount++;
						}
						else if (parameter.searchDepth > 10 && rinfo.timeLeft < course.thinkTime * (goaltime - rinfo.stepNumber) / goaltime) {
							parameter.searchDepth = 10;
							slowcount++;
						}

					}
					// ゴールを考慮した評価を計算する
					bool iscalculated = false;
					// 幅が20で長さが6のゴールをチェック済かどうか
					// そのパターンは3つあり、計算時間が大幅に増えるので、それらはすべて平地とみなして計算することにする
					bool is6calculated = false;
					for (auto& mp : rinfo.mappatterns[course.width]) {
						if (mp.type != MAPPATTERN::LOGO) {
							continue;
						}
						if (unknowny >= mp.length - 1) {
							cerr << mp.length << "," << unknowny << endl;
							// ターン開始時の初期処理を行う（置換表のクリア）
							gamedata.turn_init();
							// 元々の視界外の地形をすべて平地とする
							for (int y = rinfo.sikaiy + 1; y < course.length; y++) {
								for (int x = 0; x < course.width; x++) {
									rinfo.squares[y][x] = SquareInfo::Plain;
									for (int i = 0; i < BB_HEIGHT; i++) {
										rinfo.obst[i].reset(x, y + i);
									}
								}
							}
							// 幅が20で、長さが6のものを計算済の場合は飛ばす
							if (course.length == 20 && mp.length == 6 && is6calculated) {
								continue;
							}
							// 幅が20で長さが6のものは、すべて平地として計算する
							if (!(course.length == 20 && mp.length == 6)) {
								// ゴールの地形を設定する
								for (int y = course.length - mp.length; y < course.length; y++) {
									for (int x = 0; x < course.width; x++) {
										rinfo.squares[y][x] = mp.squares[y - (course.length - mp.length)][x];
										// 障害物だった場合は、障害物のビットボードを設定
										// ビットボード1つの高さの数だけ、y座標を一つずつずらしたものを設定する
										for (int i = 0; i < BB_HEIGHT; i++) {
											if (rinfo.squares[y][x] == SquareInfo::Obstacle) {
												rinfo.obst[i].set(x, y + i);
											}
										}
									}
								}
							}
							// 各マスのゴールからの距離を再計算する
							rinfo.calcdistance();

							// myplan2 で行動を決定する
							myplan2(0, -HYOUKA_INF, HYOUKA_INF);
							iscalculated = true;
							// 幅が20で長さが6かどうか
							bool isw20l6 = course.width == 20 && mp.length == 6;
							Hyouka mul = 1.0;
							// その場合は3つパターンがあるので、結果を3倍する
							if (isw20l6) {
								mul = 3.0;
								is6calculated = true;
							}
							// プレーヤーの各行動に対する評価値を加算する
							for (int y = -1; y <= 1; y++) {
								for (int x = -1; x <= 1; x++) {
									if (gamedata.depth0hyouka[x + 1][y + 1][1] == -HYOUKA_INF) {
										gamedata.depth0hyouka[x + 1][y + 1][1] = gamedata.depth0hyouka[x + 1][y + 1][0] * mul;
									}
									else {
										gamedata.depth0hyouka[x + 1][y + 1][1] += gamedata.depth0hyouka[x + 1][y + 1][0] * mul;
									}
								}
							}
						}
					}
					if (iscalculated) {
						//cerr << "h " << endl;
						// 評価値の計算
						Hyouka maxhyouka = -HYOUKA_INF;
						Hyouka hyouka;
						// その中の最大値を取るものを採用する
						for (int y = 1; y >= -1; y--) {
							for (int x = -1; x <= 1; x++) {
								hyouka = gamedata.depth0hyouka[x + 1][y + 1][1];
								if (hyouka > maxhyouka) {
									maxhyouka = hyouka;
									// bestaccの 更新
									gamedata.bestmove[0][0].acceleration.set(x, y);
								}
							}
						}
						//if (rinfo.stepNumber == 35) {
						//	rinfo.coursedump();
						//	rinfo.dumpdist();
						//}

						value = maxhyouka;
					}
					else {
						value = myplan2(0, -HYOUKA_INF, HYOUKA_INF);
					}
				}
				// 幅が10未満、15未満、20未満、20それぞれについて、ゴール前の予測を行う（このコードは最終的にはほぼ使っていない）
				else if (parameter.checkpattern && unknowny > 0 &&
					     ((unknowny <= parameter.maxgoalpat_w9[parameter.teban] && course.width < 10) ||
						  (unknowny <= parameter.maxgoalpat_w14[parameter.teban] && 10 <= course.width && course.width < 15) ||
						  (unknowny <= parameter.maxgoalpat_w19[parameter.teban] && 15 <= course.width && course.width < 20) ||
						  (unknowny >= 9 && unknowny <= parameter.maxgoalpat_w20[parameter.teban] && 15 <= course.width && course.width == 20))) {
					// ゴールに関する評価を行ったことを表すデバッグ表示
					cerr << "cg " << unknowny << endl;
					// 長さが9のゴール前の地形を検索する
					MAPPATTERN goalmp;
					// 長さが9のロゴのパターンを取得する
					int searchlength;
					if (course.width < 10) {
						if (unknowny < 20) {
							searchlength = 9;
						}
						else {
							searchlength = 20;
						}
					}
					else if (course.width < 15) {
						if (unknowny < 16) {
							searchlength = 9;
						}
						else if (unknowny < 20) {
							searchlength = 16;
						}
						else {
							searchlength = 20;
						}
					}
					else if (course.width < 20)  {
						if (unknowny < 9) {
							searchlength = 6;
						}
						// 長さが8のパターンは障害物がないので無視する（考慮に入れるとタイムが悪化する）
						//if (unknowny < 9) {
						//	searchlength = 8;
						//}
						else if (unknowny < 16) {
							searchlength = 9;
						}
						else if (unknowny < 20) {
							searchlength = 16;
						}
						else {
							searchlength = 20;
						}
					}
					else {
						if (unknowny < 16) {
							searchlength = 9;
						}
						else if (unknowny < 20) {
							searchlength = 16;
						}
						else {
							searchlength = 20;
						}
					}
					for (auto& mp : rinfo.mappatterns[course.width]) {
						if (mp.type != MAPPATTERN::LOGO) {
							continue;
						}
						if (mp.length == searchlength) {
							goalmp = mp;
							break;
						}
					}
					cerr << goalmp.length << "," << unknowny << endl;
					gamedata.turn_init();
					// 元々の視界外の地形をすべて平地とする
					for (int y = rinfo.sikaiy + 1; y < course.length; y++) {
						for (int x = 0; x < course.width; x++) {
							rinfo.squares[y][x] = SquareInfo::Plain;
							for (int i = 0; i < BB_HEIGHT; i++) {
								rinfo.obst[i].reset(x, y + i);
							}
						}
					}
					// ゴール前の地形を設定する
					for (int y = course.length - goalmp.length; y < course.length; y++) {
						for (int x = 0; x < course.width; x++) {
							rinfo.squares[y][x] = goalmp.squares[y - (course.length - goalmp.length)][x];
							// 障害物だった場合は、障害物のビットボードを設定
							// ビットボード1つの高さの数だけ、y座標を一つずつずらしたものを設定する
							for (int i = 0; i < BB_HEIGHT; i++) {
								if (rinfo.squares[y][x] == SquareInfo::Obstacle) {
									rinfo.obst[i].set(x, y + i);
								}
							}
						}
					}
					// 各マスのゴールからの距離を再計算する
					rinfo.calcdistance();
					value = myplan2(0, -HYOUKA_INF, HYOUKA_INF);
				}
				// それ以外の場合は myplan2 で行動を決定する
				else {
					value = myplan2(0, -HYOUKA_INF, HYOUKA_INF);
				}
				// 置換表に関する情報のデバッグ表示
				gamedata.rinfo.tt_table.dump_count();
			}
			accel = rinfo.me[0].acceleration = gamedata.getbestacc();
			rinfo.prevpl = rinfo.me[0];
			rinfo.prevene = rinfo.opponent[0];
			rinfo.me[0].dump();
			cerr << value << endl;
#ifdef RECORDHIST
			gamedata.dumpbestmove();
#endif
			break;
		}
		// 終了時刻の記録
		tdata.endturn(timer.time());
		cerr << tdata.currentdata.time << "/" << tdata.currentdata.totaltime << "ms " << endl;
		cerr.flush();

		// 加速度の出力
		cout << accel.x << ' ' << accel.y << endl;
		cout.flush();
		
		while (isspace(cin.peek())) cin.ignore(1);
	}
	// 置換表に関する情報のデバッグ表示
	tdata.dumpall();
	// ゲーム情報に関するデバッグ表示
	gamedata.dump();
	cerr << "result" << endl;
	cerr << course.width << "," << course.length << "," << course.vision << "," << course.stepLimit << "," << parameter.searchDepth << "," << parameter.searchDepthorig << "," << parameter.eneDepth << "," << parameter.maxspeedy << "," <<
		parameter.ehyoukamul[parameter.teban] << "," << parameter.stophyouka[parameter.teban] << "," << parameter.emul[parameter.teban] << "," <<
		parameter.dmul[parameter.teban] << "," << parameter.mulmin[parameter.teban] << "," << parameter.mulmax[parameter.teban] << "," << parameter.maxgoalpat_w9[parameter.teban] << "," << parameter.maxgoalpat_w14[parameter.teban] << "," << parameter.maxgoalpat_w19[parameter.teban] << "," << parameter.maxgoalpat_w20[parameter.teban] << "," <<
		tdata.maxtime << "," << tdata.totaltime << "," << course.thinkTime << static_cast<double>(tdata.totaltime) / static_cast<double>(tdata.turn) << endl;
	cerr << "slowcount" << endl;
	cerr << slowcount << endl;
}

