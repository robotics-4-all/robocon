"""
RoboCon API for model validation.

This module provides FastAPI endpoints for validating RoboCon models
through various input methods (text, file upload, base64).
"""

import base64
import logging
import os
import shutil
import tarfile
import uuid
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, File, HTTPException, Security, UploadFile, status, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.security import APIKeyHeader
from pydantic import BaseModel, Field

from robocon.definitions import TMP_DIR
from robocon.utils import build_model, check_model_errors
from robocon.m2t.rosgen import GeneratorROS
from robocon.m2t.ros2gen import GeneratorROS2

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# API Configuration
API_KEY = os.getenv("API_KEY", "123123")
api_keys = [API_KEY]

# FastAPI app initialization
api = FastAPI(
    title="RoboCon Validation API",
    description="API for validating RoboCon models",
    version="0.3.0",
)

api_key_header = APIKeyHeader(name="X-API-Key")


# Pydantic Models
class ValidationRequest(BaseModel):
    """Request model for text-based validation."""

    name: str = Field(..., description="Name of the model")
    model: str = Field(..., description="Model content as string")


class CodeGenerationRequest(BaseModel):
    """Request model for code generation."""

    name: str = Field(..., description="Name of the project/model")
    model: str = Field(..., description="Model content as string")


class ValidationError(BaseModel):
    """Individual validation error."""

    type: str = Field(..., description="Error type: syntax, semantic, or unknown")
    message: str = Field(..., description="Error message")
    line: int | None = Field(None, description="Line number where error occurred")
    column: int | None = Field(None, description="Column number where error occurred")


class ValidationResponse(BaseModel):
    """Response model for validation endpoints."""

    valid: bool = Field(..., description="Whether the model is valid")
    errors: list[ValidationError] = Field(default_factory=list, description="List of validation errors")
    
    # Deprecated fields for backward compatibility
    error: str | None = Field(None, description="Deprecated: error message (use errors list)")
    line: int | None = Field(None, description="Deprecated: error line (use errors list)")
    column: int | None = Field(None, description="Deprecated: error column (use errors list)")


# Security
def get_api_key(api_key_header: str = Security(api_key_header)) -> str:
    """
    Validate API key from request header.

    Args:
        api_key_header: API key from X-API-Key header

    Returns:
        The validated API key

    Raises:
        HTTPException: If API key is invalid or missing
    """
    if api_key_header in api_keys:
        logger.debug("API key validated successfully")
        return api_key_header

    logger.warning("Invalid API key attempt")
    raise HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Invalid or missing API Key",
    )


# Middleware
api.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Ensure temp directory exists
TMP_DIR_PATH = Path(TMP_DIR)
TMP_DIR_PATH.mkdir(parents=True, exist_ok=True)
logger.info(f"Temporary directory initialized: {TMP_DIR}")


# Helper Functions
def _validate_model_file(file_path: str) -> ValidationResponse:
    """
    Validate a model file using the check_model_errors function.

    Args:
        file_path: Path to the model file to validate

    Returns:
        ValidationResponse with valid flag and list of errors
    """
    errors = check_model_errors(file_path)
    
    if errors:
        logger.warning(f"Model validation found {len(errors)} error(s): {file_path}")
        
        # Convert error dicts to ValidationError models
        validation_errors = [
            ValidationError(
                type=err.get("type", "unknown"),
                message=err.get("message", ""),
                line=err.get("line"),
                column=err.get("column")
            )
            for err in errors
        ]
        
        # Populate backward compatibility fields from first error
        first_error = errors[0]
        return ValidationResponse(
            valid=False,
            errors=validation_errors,
            error=first_error.get("message"),
            line=first_error.get("line"),
            column=first_error.get("column")
        )
    else:
        logger.info(f"Model validation successful: {file_path}")
        return ValidationResponse(
            valid=True,
            errors=[],
            error=None,
            line=None,
            column=None
        )


def _cleanup_temp_file(file_path: str) -> None:
    """
    Clean up temporary file if it exists.

    Args:
        file_path: Path to the temporary file
    """
    try:
        if os.path.exists(file_path):
            os.remove(file_path)
            logger.debug(f"Cleaned up temporary file: {file_path}")
    except Exception as e:
        logger.warning(f"Failed to cleanup temporary file {file_path}: {e}")


def _generate_temp_filepath(extension: str = ".auto") -> str:
    """
    Generate a unique temporary file path.

    Args:
        extension: File extension for the temporary file

    Returns:
        Full path to the temporary file
    """
    u_id = uuid.uuid4().hex[:8]
    return os.path.join(TMP_DIR, f"model_for_validation-{u_id}{extension}")


def _generate_code_from_model(model_name: str, model_content: str) -> str:
    """
    Generate code from a model string and return the path to the generated tarball.

    Args:
        model_name: Name of the model/project
        model_content: RoboCon model content as string

    Returns:
        Path to the generated tarball (.tar.gz)

    Raises:
        HTTPException: If validation fails or generation fails
    """
    u_id = uuid.uuid4().hex[:8]
    temp_work_dir = Path(TMP_DIR) / f"codegen-{u_id}"
    temp_work_dir.mkdir(parents=True, exist_ok=True)
    
    model_file_path = temp_work_dir / "model.rbr"
    gen_output_dir = temp_work_dir / "gen"
    gen_output_dir.mkdir(parents=True, exist_ok=True)
    
    try:
        # Write model to temp file
        with open(model_file_path, "w", encoding="utf-8") as f:
            f.write(model_content)
            
        # Build model to check type and validate
        try:
            model_obj, _ = build_model(str(model_file_path))
        except Exception as e:
            logger.error(f"Model build failed for code generation: {e}")
            raise HTTPException(
                status_code=400,
                detail=f"Model validation failed: {e}"
            )
            
        # Detect generator type
        robot_type = getattr(model_obj.robot, 'type', None)
        if robot_type == 'ROS':
            logger.info(f"Using ROS generator for {model_name}")
            GeneratorROS.generate(model_obj, str(gen_output_dir))
        elif robot_type == 'ROS2':
            logger.info(f"Using ROS2 generator for {model_name}")
            GeneratorROS2.generate(model_obj, str(gen_output_dir))
        else:
            logger.error(f"Unsupported or missing robot type: {robot_type}")
            raise HTTPException(
                status_code=400,
                detail=f"Unsupported or missing robot type: {robot_type}. Must be ROS or ROS2."
            )
            
        # Create tarball
        tarball_path = Path(TMP_DIR) / f"{model_name}_generated_code-{u_id}.tar.gz"
        make_tarball(str(tarball_path), str(gen_output_dir))
        
        return str(tarball_path)
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Code generation failed: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Code generation failed: {e}"
        )
    finally:
        # Cleanup work directory (but keep tarball for response)
        if temp_work_dir.exists():
            shutil.rmtree(temp_work_dir)
            logger.debug(f"Cleaned up temporary work directory: {temp_work_dir}")


# API Endpoints
@api.post("/validate", response_model=ValidationResponse)
async def validate(
    model: ValidationRequest,
    api_key: str = Security(get_api_key),
) -> ValidationResponse:
    """
    Validate a model from text content.

    Args:
        model: Model request containing name and model content
        api_key: API key for authentication

    Returns:
        ValidationResponse with status and message

    Raises:
        HTTPException: If model is empty or validation fails
    """
    logger.info(f"Validation request received for model: {model.name}")

    if not model.model or len(model.model.strip()) == 0:
        logger.warning(f"Empty model content for: {model.name}")
        raise HTTPException(
            status_code=400,
            detail="Model content cannot be empty",
        )

    fpath = _generate_temp_filepath()

    try:
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(model.model)

        return _validate_model_file(fpath)
    finally:
        _cleanup_temp_file(fpath)


@api.post("/validate/file", response_model=ValidationResponse)
async def validate_file(
    file: UploadFile = File(...),
    api_key: str = Security(get_api_key),
) -> ValidationResponse:
    """
    Validate a model from an uploaded file.

    Args:
        file: Uploaded file containing the model
        api_key: API key for authentication

    Returns:
        ValidationResponse with status and message

    Raises:
        HTTPException: If validation fails
    """
    logger.info(f"File validation request: filename={file.filename}")

    fpath = _generate_temp_filepath()

    try:
        content = await file.read()
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(content.decode("utf-8"))

        return _validate_model_file(fpath)
    finally:
        _cleanup_temp_file(fpath)


@api.post("/transformation/code")
async def generate_code(
    request: CodeGenerationRequest,
    background_tasks: BackgroundTasks,
    api_key: str = Security(get_api_key),
):
    """
    Generate code from a model and return a tarball.

    Args:
        request: Code generation request containing name and model content
        background_tasks: FastAPI background tasks for cleanup
        api_key: API key for authentication

    Returns:
        FileResponse with the generated code tarball
    """
    logger.info(f"Code generation request received for model: {request.name}")
    
    if not request.model or len(request.model.strip()) == 0:
        raise HTTPException(
            status_code=400,
            detail="Model content cannot be empty"
        )
        
    tarball_path = _generate_code_from_model(request.name, request.model)
    
    # Add cleanup task to delete the tarball after the response is sent
    background_tasks.add_task(_cleanup_temp_file, tarball_path)
    
    return FileResponse(
        path=tarball_path,
        filename=f"{request.name}_generated_code.tar.gz",
        media_type="application/gzip"
    )


@api.post("/transformation/code/file")
async def generate_code_file(
    background_tasks: BackgroundTasks,
    file: UploadFile = File(...),
    api_key: str = Security(get_api_key),
):
    """
    Generate code from an uploaded model file and return a tarball.

    Args:
        background_tasks: FastAPI background tasks for cleanup
        file: Uploaded file containing the model
        api_key: API key for authentication

    Returns:
        FileResponse with the generated code tarball
    """
    logger.info(f"File code generation request: filename={file.filename}")

    # Use filename (without extension) as project name
    project_name = Path(file.filename).stem
    
    content = await file.read()
    model_content = content.decode("utf-8")
    
    if not model_content or len(model_content.strip()) == 0:
        raise HTTPException(
            status_code=400,
            detail="Model content cannot be empty"
        )
        
    tarball_path = _generate_code_from_model(project_name, model_content)
    
    # Add cleanup task to delete the tarball after the response is sent
    background_tasks.add_task(_cleanup_temp_file, tarball_path)
    
    return FileResponse(
        path=tarball_path,
        filename=f"{project_name}_generated_code.tar.gz",
        media_type="application/gzip"
    )


# Utility Functions
def make_tarball(fout: str, source_dir: str) -> None:
    """
    Create a gzipped tar archive from a source directory.

    Args:
        fout: Output file path for the tarball
        source_dir: Source directory to archive
    """
    logger.info(f"Creating tarball: {fout} from {source_dir}")
    try:
        with tarfile.open(fout, "w:gz") as tar:
            # Add all files in source_dir to the root of the tarball
            for item in os.listdir(source_dir):
                item_path = os.path.join(source_dir, item)
                tar.add(item_path, arcname=item)
        logger.info(f"Tarball created successfully: {fout}")
    except Exception as e:
        logger.error(f"Failed to create tarball {fout}: {e}")
        raise


def make_executable(path: str) -> None:
    """
    Make a file executable by setting appropriate permissions.

    This copies read (R) bits to execute (X) bits and applies them.

    Args:
        path: Path to the file to make executable
    """
    logger.info(f"Making file executable: {path}")
    try:
        mode = os.stat(path).st_mode
        mode |= (mode & 0o444) >> 2  # copy R bits to X
        os.chmod(path, mode)
        logger.debug(f"File permissions updated: {path}")
    except Exception as e:
        logger.error(f"Failed to make file executable {path}: {e}")
        raise
