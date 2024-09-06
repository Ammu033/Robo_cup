__import__('pysqlite3')
import sys
sys.modules['sqlite3'] = sys.modules.pop('pysqlite3')

from llama_index.core import VectorStoreIndex, SimpleDirectoryReader, Settings, Document
from llama_index.embeddings.huggingface import HuggingFaceEmbedding
from llama_index.llms.ollama import Ollama
from llama_index.readers.file import MarkdownReader
from llama_index.readers.database import DatabaseReader
from llama_index.vector_stores.chroma import ChromaVectorStore
from llama_index.core import StorageContext
from make_vector_embeddings import BASE_INFO_PATH, EMBEDDINGS_MODEL, EMBEDDINGS_PATH
import chromadb
import pandas
import sys


Settings.embed_model = EMBEDDINGS_MODEL
Settings.llm = Ollama(base_url="http://192.168.69.54:11434", model="llama3.1", request_timeout=150.0)

db = chromadb.PersistentClient(path=EMBEDDINGS_PATH)
chroma_collection = db.get_or_create_collection(BASE_INFO_PATH)
vector_store = ChromaVectorStore(chroma_collection=chroma_collection)
storage_context = StorageContext.from_defaults(vector_store=vector_store)


index = VectorStoreIndex.from_vector_store(
    vector_store, storage_context=storage_context
)

query_engine = index.as_query_engine()
response = query_engine.query(input("Input text-based RAG query: ")).response
print(response)

with open("../contexts/rag_query_output.txt", "w") as f:
    f.write(response)