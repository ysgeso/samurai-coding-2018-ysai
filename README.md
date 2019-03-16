# samurai-coding-2018-ysai

samuraicoding 2018 決勝に提出したAIです。
多くの参加者がはまった、コースのルールの予期せぬ変更のため、本選では自分のAIは途中で動かなくなって敗退しましたが、ここに提出したものは、コメント以外は決勝に提出したもの、そのままです。

決勝でも動くようにするには下記の変更を行えばよいと思います（試してはいませんが・・・）

* raceinfo.hpp の MAX_LENGTH を 200 に、BB_NUM を 32 に変える
* 決勝でのコース予測は無意味だったので、run.shの -cp と -gc オプションを削除する

あと、AI実行時にいろんなパラメータを設定できるようになってますが、デバッグ関連のパラメータは、途中で色々変えているうちにほぼ無意味になってます。特に、記録を取る機能は決勝時に使ったパラメータではほぼ無意味です。

run.shにはかなりたくさんパラメータが指定してありますが、そのうち2つ数字のパラメータが書かれているものは、決勝のAIでは使われていません（ソースコード内にパラメータが指定されています）。

残念ながら決勝ではまともに動きませんでしたが、せっかくなのでやったことの解説も書いてみようと思います（コード自体の可読性はあまり高くないと思いますので）。あと、当日、用事があって参加できず、決勝進出者との交流ができなかったので、誰にも自分がやったことを伝えられずせっかく苦労して作ったAIが埋もれてしまうのがちょっと悲しいので・・・

1．コースアウト判定の高速化

サンプルのコースアウト判定は、サンプルだけあってあまり高速なものとは言えないと思います。コースアウト判定の高速化は、AIの思考速度に大きく影響する部分なので、ここを高速化するのが重要だと思いました。samurai-coding-2017の時もこの部分は工夫しましたが、今回は前回とコースの障害物の仕様が大きく変わったので、一から作る必要がありました。基本的な考え方は以下の通りです。

* マップの障害物を、複数の「幅32 × 長さ8の256ビットのビットマップ」で表現する。
 
* 自分が(x, 0) の座標（０<=x<=19）にいた時に速度(dx, dy) (-5<=dx<=5, -2<=dy<=14)で移動した時の軌跡を表すビットマップをあらかじめ作成しておく（y>8の場合は、2つのビットマップで表現する）。

* 衝突判定は、マップの障害物のビットマップと、移動時の軌跡のビットマップのANDを取って、すべてのビットマップが0でなければ衝突している。

* 軌跡のビットマップはy座標が0の場合だが、任意のy座標のものを作るのは無駄なので、障害物のコースのビットマップを作る際に、y座標を 1~7までそれぞれずらしたものを作り、それと軌跡のビットマップのANDをとればうまくいく。

* 256ビットのANDはSIMD命令を使って高速に行う。

詳しくはコードを見てもらうとして、これでコード内のbenchmark関数を使って計測してみた所、サンプルと比べて約30倍くらい早くなりました。この効果はかなり大きく、おかげでゲーム木の探索時に深さ20の全探索を行う事が可能になりました。

なお、敵と味方の移動の交差判定は、特殊なことはしていません。ネットで見つけた交差判定の式をそのまま使いました。これは、後述のように、敵の行動を3手目までしか考慮しないため、交差判定の計算回数がそれほど多くないと思われたため、頑張って高速化してもほとんと恩恵がなさそうだったからです。

2.ゲーム木について

このゲームは同時着手のゲームですが、交互着手と仮定してmin-maxゲーム木を構築して最も評価値の高い手を選ぶようにしました。ただし、以下のような工夫を行っています。

* 敵のノードは3手目まで構築する。

3手目以降のノードは、敵、味方共に、自分しかゲームに存在しないと仮定して、最も評価値の高い評価値を取る行動を探索する。敵の行動を何手目まで考えるかについては色々試したところ、3手目までが一番強いような感じだったのでそうしました。敵の手をあまり先まで考えても意味がなさそうなゲームなので、3手目くらいが妥当かなと思われます。
  
* ゲーム木の最大深さは20とする。

色々試したところ、20を超えてもほとんど強くならないので、そこで打ち切りました。
 
* 深さ3までの敵のノードの評価値は、以下のような式で計算する。

敵の各行動（最大9種類）に対する評価値を小さい順に並べたものを e1, e2, ..., en とし、aは定数とする。

　(a^(n-1) * e1 + a ^ (n-2) * e2 + ... + en) / (a^(n-1) + a^(n-2) + ... + 1)
  
同時着手ゲームで、単純にmin-max木を使うと、相手はこちら手を見てから最善手を着手するということになるため、自身の手が卑屈な手になりがちなので、それを解消するために相手の最善手以外も考慮するように上記の式を考えてみました。aの値を1より大きくすればするほど、min-max木に近づき、1にすると敵の全評価値の 平均になります。aは後述の強化学習っぽい方法で決めました。
  
なお、この手法をとったため、αβ枝刈は使えなくなりましたが、敵のノードの深さが３、ゲーム木の深さが20の場合、全探索であるにもかかわらず、ほとんどのコースのゲームで、タイムリミットの30%くらいで余裕をもって思考を終了できました。これは確実に、コースアウト判定の高速化の恩恵だと思われます。
  
3.評価値について

ゲーム木は、敵、味方共に、視界外のマス、又はゴールした場合まで構築し、その位置にたいして下記の式で評価値を計算しました。
  
  各プレーヤーの評価値　= ゴールからの距離による評価値 + 評価値を計算した木の深さによる評価値 + α
  
  最終評価値 = 自分の評価値 - 敵の評価値 * β
  
αは、ゴールした場合などの特殊ケースによる若干の補正です。詳細はコードを見てください。敵の評価値はそのまま使うのではなく、定数βを乗算するようにして色々なβを試してみたところ、強くなる場合が多かったので、βを乗算しています。その値も後述の強化学習っぽい方法で決めました。βは敵がゴールしている場合とそうでない場合で異なる値を使ってみてはいるのですが、その効果のほどはよくわかりません。
  
ゴールからの距離は、各マスからのマンハッタン距離だとあまり正確ではないので、ダイクストラ法を使って、各マスは、（dx,dy）　（-5<=dx<=5,-5<=dy<=5）の点とつながっているとしてゴールからの距離を計算しました。

木の深さによる評価値ですが、深さに特定の定数（負の値）を乗算した値です。この値も後述の強化学習っぽい方法で決めました。
  
上記のようにすることで、最も早くゴールするのではなく、最も差をつけてゴールするような行動をとるようにしています。例えば、ゴールが1マスしかない場合で自分が僅差で先行していた場合、わざとゴール前のマスで止まるような行動をとることにより、敵をわざと自分にぶつからせて速度を0にしてゴールの時間差を稼ぐみたいな行動が実際にみられました。
  
4.置換表について

Zobristハッシュを使って各ノードにおける評価値を置換表に記録して起き、既に探索済の状況が現れた時は置換表に記録した値を使う手法をとりました。

5.枝刈について

y速度の最小値を-2,最大値を 8 として枝刈を行いました。9以上にして色々試しても時間がかかるだけでほぼ影響はなかったためそうしました。それ以外の枝刈は特に行っていません。
 
6.コース予測について

ルールには、決勝で使用するコースジェネレータ（予選と同じものを使うと明記されていた・・・）のソースコードが公開されていたことから、これは、コースジェネレーターで生成するコースを考慮に入れてAIを作れというのが出題者からのメッセージだ！と思っていたのですがふたを開けてみると、実際には明らかに予選のコースジェネレータで作られるはずのないコースが決勝で・・・上位者の多くはそれではまってたみたいですね。もちろん私のAIも・・・というわけで実際には無意味（どころか悪影響）だったのですが、せっかくなのでコース予測についても記します。主に3つを実装しました。
 
* パターンによる予測

コースジェネレータのソースコードを読むと、コースに使ういくつかのパターン地形が用意されていました。そのため、パターンの地形と同じ地形の一部が視界の端に見えた場合、その先の地形をそのパターンの地形だと仮定することができます。本AIでは、視界の端の地形がパターンの地形と幅が1マス分でも一致したらパターンの地形と仮定しました。これは確実に予選のコースジェネレータで作られたコースに対しては大きな効果をもたらしました。
  
* トラップ地形の対策

パターンの地形のうちのいくつかは、トラップ地形になっていて、そのパターンの地形の一番奥の地形だけ、ランダムで右端または左端のみ通行可能なようになっていました。トラップ地形の先をすべて見通せないうちに、左右のどちらかに進んでしまうとトラップにはまってしまうというものです。このゲームは、視界を共有するため、先行しているほうが、視界が悪く、トラップ地形にひっかかる可能性が高いのは、先行しているプレーヤーです。そのためトラップ地形はあまり大胆に先に進まないほうが良いケースが多そうでした。
  
そこで、トラップ地形を予測した場合は、右端が空いている場合と左端が空いている場合の両方に対して評価値を計算し、最終的な評価値はその平均をとるという形にしました。そうすることで、トラップ地形に先行した場合は、減速して、場合によっては敵が追い付いて、トラップ地形の先に進むまで待つような行動を取るようになりました。ただ、この行動が本当に最適なのかがよく分からなかったので、最終的には2つの評価値の大きいほうに定数を乗算することにしました。その定数も後述の強化学習っぽい方法で決めました。
  
* ゴール地形の予測

予選のコースジェネレータでは、必ずスタート地点とゴール地点はISPJまたはSAMURAIのロゴをもじった地形が生成されるようになっていました。スタート地点は必ず視界内なので通常の予測方法で良いのですが、ゴール付近に関しては、視界に入る前からある程度の予測が可能となります。また、コースの幅が10未満の場合は、ロゴの地形の種類が2種類、15未満の場合は3種類、20未満の場合は5種類、20の場合は7種類と、コースの幅が狭い場合は、かなりゴール付近の地形の種類が限定されていました。そこで、ゴール付近を視界に収めた場合は、トラップと同じように、可能性のあるゴールの地形すべてに対して評価値を計算し、その平均を取るという手法をとりました（ただし、幅が20の場合は、可能性が多すぎるので、少し簡略化して計算を行っています）。ゴールにどこまで近づいたらこのゴール予測を行うかについては、定数で定め、その定数も後述の強化学習っぽい方法で決めました。
  
決勝でスタート地点ではまってたAIはおそらくこれらの地形予測が何らかの悪影響をもたらしたことが原因なんだろうなと思います・・・

7．強化学習っぽいもの

ここまで、いくつか定数パラメータが出てきましたが、それらの値は、自己対戦で、下記のような単純な強化学習っぽい手法で決めました。
 
* 各パラメータの初期値を設定する。
* 各パラメータについて下記の作業を行う。
* 一つのパラメータを一定範囲で少しずつ変化させて最も成績のよい値を見つける
* すべてのパラメータについて上記の作業を行った後、各パラメータについて範囲を狭めて上記の作業を行うということを1~5回行う
 
最初は幅、視界に関係なく同じパラメータでやってたのですが、よく考えると、コースの幅、視界、先手、後手の違いで最適なパラメータが違うのではないかと思って、各コース幅5~20、視界5~20全てに対してコースジェネレーターで100個コースを作って上記の作業を行おうかと考えました。しかし、100コースの対戦だけで5~10分程かかることがわかり、上記の作業１つで3~6時間以上かかることがわかったため、 結局の所は 幅5,10,15,20、視界5,10,20にしぼって繰り返しは1~2回でごまかしました。あと、途中でバグ修正とかして何度も一からやり直すハメに何回もなったり・・・
 
で、そこまでして作ったパラメータ設定はソースコードに書いてある通りですが、見てもらうとわかるとおり、幅、視界が少し違うとかなりバラバラな値になってあまり法則性が見つかりませんでした。まあまあ強いことは確かなのでそのまま作ったパラメータで提出しましたが、100コースくらいでは過学習になってて一般的に強いパラメータは見つけられそうにない感じです。やっぱり学習は計算機資源がないと厳しいですね。

  
   
 

