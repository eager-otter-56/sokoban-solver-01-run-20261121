import N from"node:fs";N.readdirSync("/input").map(e=>{let t=N.readFileSync(`/input/${e}/level.txt`)+"",w=t.indexOf("\n")+1,D=[-w,w,-1,1],L="udlr",B=[],G=[],T={},P,k,x,i,K,m="",o,v,q,H,
R=(Z,p)=>{let v={[p]:""},q=[p];for(let s of q)for(k in D)x=s+D[k],T[x]&&!Z.includes(x)&&!(x in v)&&(v[x]=v[s]+L[k],q.push(x));return v};
t.replace(/[^#\n]/g,(c,i,_,j=" .$*@+".indexOf(c))=>{T[i]=1;j&1&&G.push(i);j&2&&B.push(i);j&4&&(P=i)});
H=[...Array(w)].map((_,i)=>P-P%w+i);
for(o of G){v=R(H,o);q=[[B.sort(),P,""]];for(let[b,p,s]of q){if(G.every(x=>x in v?b.includes(x):1)){m+=s;P=p;B=b;break}let W=R(b,p);for(i in b)for(k in D){let z=b[i],y=z+D[k],n=W[z-D[k]],E=[...b];z in v&&n>=""&&y in v&&!b.includes(y)&&(T[y-1]&&T[y+1]||T[y-w]&&T[y+w]||G.includes(y))&&(E[i]=y,T[K=E.sort()+"|"+z]||q.push([E,z,s+n+L[k],T[K]=1]))}}}
N.mkdirSync(e="/output/"+e,{recursive:!0});N.writeFileSync(e+"/solution.txt",m+"\n")})
