public class RandomTester {

    public Graph generateRandomGraph(int numVertices, int numEdges) {
        Graph g = new GraphAdjMatrix();
        for (int i = 0; i < numVertices; i++) {
            g.addVertex();
        }
        for (int i = 0; i < numEdges; i++) {
            int v = (int) (Math.random() * numVertices);
            int w = (int) (Math.random() * numVertices);
            g.addEdge(v, w);
        }
        return g;
    }

    public void generateTextFile(Graph graph) {

    }
}