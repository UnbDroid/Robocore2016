#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>

using namespace std;

class Objeto{
public:
	Objeto(string meu_id, string meu_simbolo, int x_ini, int y_ini){
		id = meu_id;
		simbolo = meu_simbolo;
		x = x_ini;
		y = y_ini;
	}
	string get_id() const{
		return id;
	}
	int get_x() const{
		return x;
	}
	int get_y() const{
		return y;
	}
	string get_simbolo() const{
		return simbolo;
	}
	virtual void atualizar(int & x, int & y){
		x = this->x;
		y = this->y;
	}
protected:
	string id,simbolo;
	int x, y;
};

class Mapa
{
public:
	Mapa(){

	}

	Mapa(int m, int n){
		mapa = new int [m*n];
		this->m = m;
		this->n = n;
		for (int i = 0; i < m*n; ++i){
			mapa[i] = 0;
		}
		construir_representacao();
	}

	~Mapa(){
		delete[] mapa;
	}

	string representacao() const{
		return repr;
	}

	void inserir_obstaculo(int m, int n){
		if(m >= this->m || n >= this->n || m < 0 || n < 0)
			return;
		mapa[n*this->m+m] = 1;
		repr.replace(4*(n*this->m+m)+2,1,"\u25A0",2,2);
	}

	void inserir_retangulo(int m_inicial, int n_inicial, int m_final, int n_final){
		bool cond1 = (m_inicial >= m || n_inicial >= n || m < 0 || n < 0);
		bool cond2 = (m_final >= m || n_final >= n);
		bool cond3 = (m_inicial > m_final || n_inicial > n_final);
		if(cond1 || cond2 || cond3)
			return;

		for (int i = m_inicial; i <= m_final; ++i){
			for (int j = n_inicial; j <= n_final; ++j){
				inserir_obstaculo(i,j);
			}
		}

	}

	void inserir_objeto(Objeto * obj){
		if ( objs.find(obj->get_id()) == objs.end() ) {
		  objs.insert(pair<string, Objeto*>(obj->get_id(),obj));
		}
	}

	void atualizar_mapa(){
		int x,y;
		for (map<string, Objeto*>::iterator it=objs.begin(); it!=objs.end(); ++it){
    		Objeto* obj = it->second;
    		remover_objeto(obj->get_x(),obj->get_y());
    		obj->atualizar(x,y);
    		desenhar_objeto(obj,x,y);
		}

	}

private:
	int * mapa;
	map<string, Objeto*> objs;
	int m,n;
	string repr;
	void construir_representacao(){
		repr = string();
		for (int i = 0; i < n; ++i){
			for (int j = 0; j < m; ++j){
				if(mapa[i*m+j]==0){
					repr += "\u25A1";
				}else if(mapa[i*m+j]==1){
					repr += "\u25A0";
				}else if(mapa[i*m+j]==2){
					repr += "\u25A3";
				}
				if(j<m-1)
					repr+=" ";
			}
			repr+='\n';
		}
	}

	void desenhar_objeto(Objeto* obj, int x, int y){
		if(x >= m || y >= n || x < 0 || y < 0)
			return;
		mapa[y*m+x] = 3;
		repr.replace(4*(y*m+x)+2,1,obj->get_simbolo(),2,2);
	}

	void remover_objeto(int x, int y){
		if(x >= m || y >= n || x < 0 || y < 0)
			return;
		mapa[y*m+x] = 3;
		repr.replace(4*(y*m+x)+2,1,"\u25A1",2,2);
	}
};

std::ostream &operator<<(std::ostream &os, Mapa const &m) { 
    return os << m.representacao();
}

class Coordenadas{
public:
	Coordenadas(){
		this->x = 0;
		this->y = 0; 
	}

	Coordenadas(int x,int y){
		this->x = x;
		this->y = y; 
	}

	int get_x() const{
		return x;
	}

	int get_y() const{
		return y;
	}
private:
	int x,y;
};

class Estado{
public:
	Estado(){
		coord = Coordenadas();
	}
	Estado(int pos_x, int pos_y){
		coord = Coordenadas(pos_x, pos_y);
	}

	Estado(Coordenadas coord){
		this->coord = coord;
	}

	Coordenadas get_coord() const{
		return coord;
	}
private:
	Coordenadas coord;
};


class Robo: public Objeto
{
public:
	Robo(string nome_robo, Mapa & mapa_inicial,int pos_x_inicial ,int pos_y_inicial, string simbolo="\u25A3"):
	Objeto(nome_robo, simbolo, pos_x_inicial, pos_y_inicial),meu_mapa(mapa_inicial){
		estado_atual = Estado(pos_x_inicial, pos_y_inicial);
		meu_mapa.inserir_objeto(this);
		definir_velocidade(0,0);
	}
	void atualizar(int & x, int & y){
		calcular_estado_atual();
		x = estado_atual.get_coord().get_x();
		y = estado_atual.get_coord().get_y();
		this->x=x;
		this->y=y;
	}
	void definir_velocidade(int v_x, int v_y){
		this->v_x = v_x;
		this->v_y = v_y;
	}
private:
	Mapa & meu_mapa;
	Estado estado_atual;
	int v_x,v_y;
	void calcular_estado_atual(){
		int x,y;
		x = estado_atual.get_coord().get_x();
		y = estado_atual.get_coord().get_y();
		estado_atual = Estado(x+v_x,y+v_y);
	}
};


int main(int argc, char const *argv[])
{
	//Mapa mapa_real(10,10);
	//cout << mapa_real << endl;
	//mapa_real.inserir_obstaculo(4,4);
	//mapa_real.inserir_retangulo(1,2,1,3);
	Mapa mapa_robo(10,10);
	mapa_robo.inserir_retangulo(1,2,1,3);
	//cout << mapa_real << endl;
	Robo bruce("Bruce", mapa_robo, 0,0);
	//cout << mapa_robo;
	//string banana = "\u25A1 \u25A1 \u25A1\n\u25A1 \u25A1 \u25A1";
	//cout << banana << endl << endl;
	//banana.replace(2,1,"\u25A0",2,2);
	//cout << banana << endl;
	mapa_robo.atualizar_mapa();
	cout << mapa_robo << endl;
	bruce.definir_velocidade(1,0);
	for (int i = 0; i < 4; ++i){
		system("clear");
		mapa_robo.atualizar_mapa();
		cout << mapa_robo << endl;
		sleep(1);
	}
	bruce.definir_velocidade(0,1);
	for (int i = 0; i < 4; ++i){
		system("clear");
		mapa_robo.atualizar_mapa();
		cout << mapa_robo << endl;
		sleep(1);
	}
	return 0;
}