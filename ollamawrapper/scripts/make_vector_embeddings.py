__import__('pysqlite3')
import sys
sys.modules['sqlite3'] = sys.modules.pop('pysqlite3')

from llama_index.core import VectorStoreIndex, SimpleDirectoryReader, Settings, Document
from llama_index.embeddings.huggingface import HuggingFaceEmbedding
from llama_index.llms.ollama import Ollama
from llama_index.readers.file import MarkdownReader
from llama_index.readers.database import DatabaseReader
from llama_index.readers.docstring_walker import DocstringWalker
import sqlalchemy
import pandas
import chromadb
from llama_index.core import VectorStoreIndex, SimpleDirectoryReader
from llama_index.vector_stores.chroma import ChromaVectorStore
from llama_index.core import StorageContext
import database
import subprocess
import sys
import os

BASE_INFO_PATH = "Eindhoven2024"
EMBEDDINGS_MODEL = HuggingFaceEmbedding(model_name="BAAI/bge-base-en-v1.5")
SQLITE_PATH = "../contexts/%s_objects.db" % BASE_INFO_PATH
EMBEDDINGS_PATH = "../contexts/%s_vectors.chroma_db" % BASE_INFO_PATH

def main():
    subprocess.run(["rm", "-rf", EMBEDDINGS_PATH])
    subprocess.run(["rm", "-f", SQLITE_PATH])

    Settings.embed_model = EMBEDDINGS_MODEL

    input_files = [
        "%s/names/names.md" % BASE_INFO_PATH,
        "%s/questions/questions.md" % BASE_INFO_PATH,
        "%s/maps/room_names.md" % BASE_INFO_PATH,
        # "%s/objects/objects.md" % BASE_INFO_PATH,
        # "%s/maps/location_names.md" % BASE_INFO_PATH
    ]

    parser = MarkdownReader()
    file_extractor = {".md": parser}
    documents = SimpleDirectoryReader(input_files=input_files, file_extractor=file_extractor).load_data()

    with database.ObjectsLocationsDB(base_info_path = BASE_INFO_PATH, db_path = SQLITE_PATH) as db:
        engine = sqlalchemy.create_engine('sqlite:///%s' % db.db_path)
    q = """SELECT objects.object_name AS object, object_classes.class_name AS class, locations.location_name AS locations 
    FROM objects 
    INNER JOIN object_classes ON objects.class_id = object_classes.class_id 
    INNER JOIN locations ON locations.class_id = objects.class_id;"""

    # test
    print(pandas.read_sql(q, engine))

    reader = DatabaseReader(engine=engine)
    documents += reader.load_data(query=q)
    documents += reader.load_data(query="SELECT location_name AS all_locations FROM locations;")

    documents += [DocstringWalker().parse_module("gpsr", "../src/capabilities/rag_gpsr.py")]

    db = chromadb.PersistentClient(path=EMBEDDINGS_PATH)
    chroma_collection = db.get_or_create_collection(BASE_INFO_PATH)

    vector_store = ChromaVectorStore(chroma_collection=chroma_collection)
    storage_context = StorageContext.from_defaults(vector_store=vector_store)

    index = VectorStoreIndex.from_documents(
        documents, storage_context=storage_context
    )

if __name__ == "__main__":
    main()
    print("RAG Vector Embeddings successfully made. If you change GPSR capabilities, you need to remake them.")