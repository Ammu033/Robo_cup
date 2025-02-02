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
import argparse
import chromadb
import pandas
import sys

parser = argparse.ArgumentParser()
parser.add_argument(
    "-m", "--model",
    type = str,
    default = "llama3.1"
)
parser.add_argument(
    "--host",
    type = str,
    default = "http://127.0.0.1:11434"
)
parser.add_argument(
    "-q", "--query",
    type = str,
    required = False
)
args = vars(parser.parse_args())


Settings.embed_model = EMBEDDINGS_MODEL
# Settings.llm = Ollama(base_url="http://127.0.0.1:11434", model="deepseek-r1:8b", request_timeout=150.0)
Settings.llm = Ollama(base_url=args["host"], model=args["model"], request_timeout=150.0)

db = chromadb.PersistentClient(path=EMBEDDINGS_PATH)
chroma_collection = db.get_or_create_collection(BASE_INFO_PATH)
vector_store = ChromaVectorStore(chroma_collection=chroma_collection)
storage_context = StorageContext.from_defaults(vector_store=vector_store)


index = VectorStoreIndex.from_vector_store(
    vector_store, storage_context=storage_context
)

query_engine = index.as_query_engine()
if args["query"] is None:
    query = input("Input text-based RAG query: ")
else:
    query = args["query"]

response = query_engine.query(query)

print(response)

# with open("../contexts/rag_query_output.txt", "w") as f:
#     f.write(response)